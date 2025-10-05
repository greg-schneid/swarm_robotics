import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32MultiArray, Bool

from typing import Callable, List, Optional, Tuple

import numpy as np
from scipy.signal import iirnotch, butter, filtfilt, welch, detrend

# Import ROS2 custom messages
from swarm_messages.msg import BrainData, BrainActionMsg


# ---------------------------------------------------------------------
# DSP helpers
# ---------------------------------------------------------------------

def _bandpass(sig: np.ndarray, fs: float, lo: float, hi: float, order: int = 4) -> np.ndarray:
    b, a = butter(order, [lo/(fs/2), hi/(fs/2)], btype='band')
    return filtfilt(b, a, detrend(sig, type='linear'))

def _notch(sig: np.ndarray, fs: float, mains: float = 60.0, Q: float = 30.0) -> np.ndarray:
    b, a = iirnotch(w0=mains/(fs/2), Q=Q)
    return filtfilt(b, a, sig)

def _bandpower_welch(sig: np.ndarray, fs: float, f_lo: float, f_hi: float) -> float:
    # 2-second Welch for decent resolution; adjust nperseg if fs differs
    nps = max(64, int(fs * 2))
    f, pxx = welch(sig, fs=fs, nperseg=nps, noverlap=nps // 2, window="hann")
    mask = (f >= f_lo) & (f < f_hi)
    if not np.any(mask):
        return 0.0
    return float(np.trapz(pxx[mask], f[mask]))


# ---------------------------------------------------------------------
# BrainProcessor
# ---------------------------------------------------------------------

class BrainProcessor(Node):
    """
    Real-time mental-state decoder (Relax vs Focus) with calibration.
    - Maintains a rolling buffer (seconds) of EEG
    - Filters (1–45 Hz) + Notch
    - Computes Alpha (TP9/TP10, 8–13 Hz), Beta (AF7/AF8, 13–30 Hz)
    - Uses Beta/Alpha ratio + hysteresis → actions
    """

    def __init__(
        self,
        sample_rate: float = 256.0,
        labels: Optional[List[str]] = None,           # default Muse order used if None
        buffer_sec: float = 4.0,                       # processing window
        mains_hz: float = 60.0,                       # set 50.0 if needed
        action_callback: Optional[Callable[[BrainActionMsg], None]] = None,
        use_default_thresholds: bool = False,          # use defaults immediately
    ):
        super().__init__('brain_processor')
        self.fs = float(sample_rate)
        self.labels = labels if labels else ["TP9", "AF7", "AF8", "TP10", "AUX"]
        self.mains_hz = mains_hz
        self.use_default_thresholds = use_default_thresholds

        self.S = int(max(1, round(buffer_sec * self.fs)))  # samples in ring buffer
        self.C = len(self.labels)
        self._rb = np.zeros((self.S, self.C), dtype=np.float64)
        self._write = 0
        self._filled = 0
        self._last_data_time: Optional[float] = None
        self._data_stale_warned = False

        # thresholds learned in calibrate() or set to defaults
        if use_default_thresholds:
            # Default thresholds based on typical Beta/Alpha ratios
            # These work reasonably well but calibration is recommended for best results
            self.r_low: Optional[float] = 0.8    # Below this = STOP (very relaxed)
            self.r_high: Optional[float] = 1.5   # Above this = FORWARD (focused)
            self.r_boost: Optional[float] = 2.5  # Above this = BOOST (highly focused)
            self.get_logger().info(
                f"Using default thresholds: low={self.r_low:.2f}, high={self.r_high:.2f}, boost={self.r_boost:.2f}"
            )
        else:
            self.r_low: Optional[float] = None
            self.r_high: Optional[float] = None
            self.r_boost: Optional[float] = None
            self.get_logger().info("No thresholds set. Calibrate should run before processing.")

        # bands
        self.alpha = (8.0, 13.0)
        self.beta  = (13.0, 30.0)

        # outputs
        self._state = "IDLE"
        self._action_cb = action_callback

        # user-friendly mapping if you want to expose human labels
        self.action_map = {
            "Relax":  "STOP",
            "Focus":  "FORWARD",
            "Boost":  "BOOST",
            "Idle":   "IDLE",
        }

        self.brain_data_sub = self.create_subscription(BrainData, '/brain/raw', self.receive_data, 100)
        self.brain_action_pub = self.create_publisher(BrainActionMsg, '/brain/action', 10)
        self.brainaction_metrics_pub = self.create_publisher(Float32MultiArray, '/brain/metrics', 10)
        self.connected_sub = self.create_subscription(Bool, '/brain/connected', self._on_connected, 10)

        self.create_timer(1.0 / 5.0, self.process_tick)

        # Calibration settings
        self.attempted_calibration = False
        self.calibrating = False
        self._calibration_buffer: List[np.ndarray] = []  # Stores samples during calibration
    
    def _on_connected(self, msg: Bool) -> None:
        if msg.data and not self.attempted_calibration and not self.calibrating and not self.use_default_thresholds:
            self.get_logger().info("Brain device connected. Starting calibration...")
            self.calibrate()

    def receive_data(self, data: BrainData) -> None:
        """Push a sample (or small chunk shaped (C,)) into the ring buffer."""
        v = np.asarray(data.values, dtype=np.float64).reshape(-1)
        if v.size != self.C:
            # tolerate AUX absence (C=4)
            if v.size == self.C - 1:
                v = np.concatenate([v, [0.0]])
            else:
                # wrong shape, drop
                return
        
        # If calibrating, also store in calibration buffer
        if self.calibrating:
            self._calibration_buffer.append(v.copy())
        
        self._push_row(v)
        self._last_data_time = self._now()
        self._data_stale_warned = False

    def process_tick(self) -> None:
        """
        Call this at ~5–10 Hz (yours: 5 Hz). Computes features over the current
        window and emits actions according to thresholds (after calibration).
        """
        if self._filled < int(self.fs * 1.5):  # wait until ~1.5s buffered
            return

        # Guard against stale feed
        if self._last_data_time is not None:
            stale_sec = self._now() - self._last_data_time
            if stale_sec > 2.0:
                if not self._data_stale_warned:
                    self.get_logger().warning(
                        f"Brain data stream stale for {stale_sec:.1f}s; skipping processing."
                    )
                    self._data_stale_warned = True
                return
            else:
                self._data_stale_warned = False

        alpha, beta, ratio = self._summarize_ratio(self._window())

        # If not calibrated yet, log the ratio but don't emit actions
        if self.r_low is None or self.r_high is None:
            self.get_logger().info(
                f"Not calibrated. Current ratio={ratio:.2f}, alpha={alpha:.1f}, beta={beta:.1f}"
            )
            return

        # Decide action with hysteresis
        new_state = self._state
        if ratio > (self.r_boost or 9e9):
            new_state = "BOOST"
        elif ratio > (self.r_high or 9e9):
            new_state = "FORWARD"
        elif ratio < (self.r_low or -9e9):
            new_state = "STOP"
        else:
            new_state = "IDLE"
        
        # Update state and always publish (not just on state changes)
        self._state = new_state
        msg = BrainActionMsg()
        msg.stamp = float(self.get_clock().now().nanoseconds) * 1e-9
        msg.action = new_state
        msg.beta_alpha_ratio = float(ratio)
        msg.alpha_power = float(alpha)
        msg.beta_power = float(beta)
        self._emit(msg)
    
    def calibrate(
        self,
        relaxed_sec: float = 10.0,
        focus_sec: float = 10.0,
    ) -> Tuple[float, float, float]:
        """
        Non-blocking calibration:
          1) Relaxed (eyes closed) relaxed_sec seconds
          2) Focus (mental math) focus_sec seconds
        Returns (r_low, r_high, r_boost).
        """
        self.calibrating = True
        
        # Phase 1: Relaxed
        min_relaxed_samples = max(10, int(relaxed_sec * self.fs * 0.5))
        relaxed_ratios = self._collect_phase_ratios(
            phase_name="RELAXED",
            phase_index=1,
            target_duration=relaxed_sec,
            min_samples=min_relaxed_samples
        )

        # Phase 2: Focus
        min_focus_samples = max(10, int(focus_sec * self.fs * 0.5))
        focus_ratios = self._collect_phase_ratios(
            phase_name="FOCUS",
            phase_index=2,
            target_duration=focus_sec,
            min_samples=min_focus_samples
        )

        if len(relaxed_ratios) < 5 or len(focus_ratios) < 5:
            self.get_logger().warning(
                f"Calibration aborted: collected {len(relaxed_ratios)} relaxed and {len(focus_ratios)} focus "
                f"ratios (need at least 5 each)."
            )
            self.calibrating = False
            return float('nan'), float('nan'), float('nan')

        r_relax = float(np.median(relaxed_ratios))
        r_focus = float(np.median(focus_ratios))

        # thresholds with buffer (hysteresis)
        self.r_low   = (r_relax * 0.9 + r_focus * 0.1)
        self.r_high  = (r_relax * 0.3 + r_focus * 0.7)
        self.r_boost = (r_focus * 1.2)
        self.attempted_calibration = True
        self.calibrating = False

        self.get_logger().info(
            f"[BrainProcessor] Calibration complete. "
            f"Relaxed ratio ~{r_relax:.2f}, Focused ratio ~{r_focus:.2f}. "
            f"Set thresholds: low={self.r_low:.2f}, high={self.r_high:.2f}, boost={self.r_boost:.2f}"
        )

        return self.r_low, self.r_high, self.r_boost

    # ----------------- Internals -----------------

    def _idx(self, name: str) -> int:
        mapping = {n: i for i, n in enumerate(self.labels)}
        # fallbacks for default Muse
        defaults = {"TP9": 0, "AF7": 1, "AF8": 2, "TP10": 3, "AUX": 4}
        return mapping.get(name, defaults[name])

    def _summarize_ratio(self, win: np.ndarray) -> Tuple[float, float, float]:
        fs = self.fs
        # filter per channel
        filt = np.empty_like(win)
        for c in range(win.shape[1]):
            x = win[:, c]
            x = _bandpass(x, fs, 1.0, 45.0)
            x = _notch(x, fs, self.mains_hz)
            filt[:, c] = x

        tp9  = filt[:, self._idx("TP9")]
        tp10 = filt[:, self._idx("TP10")]
        af7  = filt[:, self._idx("AF7")]
        af8  = filt[:, self._idx("AF8")]

        alpha = 0.5 * (_bandpower_welch(tp9, fs, *self.alpha) +
                       _bandpower_welch(tp10, fs, *self.alpha))
        beta  = 0.5 * (_bandpower_welch(af7, fs, *self.beta)  +
                       _bandpower_welch(af8, fs, *self.beta))

        ratio = beta / (alpha + 1e-9)
        return alpha, beta, ratio

    def _push_row(self, row: np.ndarray) -> None:
        S = self.S
        self._rb[self._write, :] = row
        self._write = (self._write + 1) % S
        self._filled = min(S, self._filled + 1)

    def _window(self) -> np.ndarray:
        if self._filled < self.S:
            return self._rb[:self._filled, :]
        if self._write == 0:
            return self._rb
        return np.vstack((self._rb[self._write:, :], self._rb[:self._write, :]))

    def _emit(self, msg: BrainActionMsg) -> None:
        if self._action_cb:
            self._action_cb(msg)
        
        # Publish the action message
        self.brain_action_pub.publish(msg)
        
        # Publish metrics as Float32MultiArray
        metrics = Float32MultiArray()
        metrics.data = [msg.beta_alpha_ratio, msg.alpha_power, msg.beta_power]
        self.brainaction_metrics_pub.publish(metrics)
        
        self.get_logger().debug(
            f"[BrainProcessor] {msg.action:7s}  R={msg.beta_alpha_ratio:5.2f}  "
            f"α={msg.alpha_power:7.1f}  β={msg.beta_power:7.1f}"
        )

    def _now(self) -> float:
        """Return current time in seconds."""
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _wait_for_buffer(self, min_seconds: float, timeout: float) -> bool:
        """Block until at least `min_seconds` of samples are buffered or timeout occurs."""
        if min_seconds <= 0:
            return True

        needed_samples = int(self.fs * min_seconds)
        start = self._now()

        while rclpy.ok() and self._filled < needed_samples:
            if (self._now() - start) >= timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)

        return self._filled >= needed_samples

    def _collect_phase_ratios(
        self,
        phase_name: str,
        phase_index: int,
        target_duration: float,
        min_samples: int,
    ) -> List[float]:
        """Collect beta/alpha ratios for a calibration phase by waiting then computing from buffer."""
        # Clear calibration buffer for this phase
        self._calibration_buffer.clear()
        
        self.get_logger().info(
            f"[BrainProcessor] Prepare for phase {phase_index}: {phase_name}."
        )
        
        # Countdown
        for i in range(3, 0, -1):
            self.get_logger().info(
                f"[BrainProcessor] Calibration phase {phase_index}: {phase_name} starting in {i}..."
            )
            self.get_clock().sleep_for(Duration(seconds=1))
            rclpy.spin_once(self, timeout_sec=0.0)

        # Start collecting data
        self.calibrating = True
        phase_start = self._now()
        
        self.get_logger().info(
            f"[BrainProcessor] Collecting {phase_name} data for {target_duration}s..."
        )
        
        # Wait for the full duration while spinning to receive data
        while rclpy.ok() and (self._now() - phase_start) < target_duration:
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop collecting for this phase
        self.calibrating = False
        
        # Now compute ratios from the collected buffer
        ratios: List[float] = []
        if len(self._calibration_buffer) < min_samples:
            self.get_logger().warning(
                f"[BrainProcessor] Phase {phase_index} ({phase_name}) captured {len(self._calibration_buffer)} samples; "
                f"needed {min_samples}."
            )
            return ratios
        
        # Convert buffer to numpy array
        cal_data = np.array(self._calibration_buffer)
        
        # Compute ratios using sliding windows
        window_size = int(2 * self.fs)  # 2 second windows
        step_size = int(0.5 * self.fs)  # 0.5 second steps
        
        for i in range(0, len(cal_data) - window_size + 1, step_size):
            window = cal_data[i:i+window_size, :]
            if window.shape[0] >= window_size:
                _, _, ratio = self._summarize_ratio(window)
                ratios.append(float(ratio))
        
        self.get_logger().info(
            f"[BrainProcessor] Phase {phase_index} ({phase_name}) computed {len(ratios)} ratios from {len(cal_data)} samples."
        )
        
        return ratios


def main(args=None):
    """Main entry point for the brain_processor node."""
    rclpy.init(args=args)
    node = BrainProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
