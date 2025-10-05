import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

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
    ):
        super().__init__('brain_processor')
        self.fs = float(sample_rate)
        self.labels = labels if labels else ["TP9", "AF7", "AF8", "TP10", "AUX"]
        self.mains_hz = mains_hz

        self.S = int(max(1, round(buffer_sec * self.fs)))  # samples in ring buffer
        self.C = len(self.labels)
        self._rb = np.zeros((self.S, self.C), dtype=np.float64)
        self._write = 0
        self._filled = 0

        # thresholds learned in calibrate()
        self.r_low: Optional[float] = None
        self.r_high: Optional[float] = None
        self.r_boost: Optional[float] = None

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

        self.create_timer(1.0 / 5.0, self.process_tick)

        # Calibration settings
        self.attempted_calibration = False
        

    # ----------------- Public API -----------------

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
        self._push_row(v)

    def process_tick(self) -> None:
        """
        Call this at ~5–10 Hz (yours: 5 Hz). Computes features over the current
        window and emits actions according to thresholds (after calibration).
        """
        if self._filled < int(self.fs * 1.5):  # wait until ~1.5s buffered
            return

        alpha, beta, ratio = self._summarize_ratio(self._window())

        # If not calibrated yet, just print/debug
        if self.r_low is None or self.r_high is None:
            # no action yet
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

        if new_state != self._state:
            self._state = new_state
            msg = BrainActionMsg()
            msg.stamp = self._now()
            msg.action = new_state
            msg.beta_alpha_ratio = float(ratio)
            msg.alpha_power = float(alpha)
            msg.beta_power = float(beta)
            self._emit(msg)
    
    def calibrate(
        self,
        relaxed_sec: float = 10.0,
        focus_sec: float = 10.0,
        interactive: bool = True,
        logger: Optional[Callable[[str], None]] = None,
    ) -> Tuple[float, float, float]:
        """
        Blocking calibration:
          1) Relaxed (eyes closed) relaxed_sec seconds
          2) Focus (mental math) focus_sec seconds
        Returns (r_low, r_high, r_boost).
        """
        log = logger if logger else print
        
        # Accumulate ratios during two phases
        relaxed_ratios: List[float] = []
        focus_ratios:   List[float] = []

        # phase 1: relaxed
        if interactive:
            input(f"[BrainProcessor] Calibration phase 1: RELAXED for ~{relaxed_sec:.1f} s. Enter when ready...")
        else:
            log(f"[BrainProcessor] Calibration phase 1: RELAXED for ~{relaxed_sec:.1f} s starting now.")
        
        t0 = self._now()
        while self._now() - t0 < relaxed_sec:
            if self._filled >= int(2 * self.fs):
                _, _, r = self._summarize_ratio(self._window())
                relaxed_ratios.append(float(r))

        # phase 2: focus
        if interactive:
            input(f"[BrainProcessor] Calibration phase 2: FOCUS for ~{focus_sec:.1f} s. Enter when ready...")
        else:
            log(f"[BrainProcessor] Calibration phase 2: FOCUS for ~{focus_sec:.1f} s starting now.")

        t0 = self._now()
        while self._now() - t0 < focus_sec:
            if self._filled >= int(2 * self.fs):
                _, _, r = self._summarize_ratio(self._window())
                focus_ratios.append(float(r))

        if not relaxed_ratios or not focus_ratios:
            raise RuntimeError("Calibration failed: insufficient buffered data.")

        r_relax = float(np.median(relaxed_ratios))
        r_focus = float(np.median(focus_ratios))

        # thresholds with buffer (hysteresis)
        self.r_low   = (r_relax * 0.9 + r_focus * 0.1)
        self.r_high  = (r_relax * 0.3 + r_focus * 0.7)
        self.r_boost = (r_focus * 1.2)

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
        
        self.get_logger().info(
            f"[BrainProcessor] {msg.action:7s}  R={msg.beta_alpha_ratio:5.2f}  "
            f"α={msg.alpha_power:7.1f}  β={msg.beta_power:7.1f}"
        )

    def _now(self) -> float:
        """Return current time in seconds."""
        return float(self.get_clock().now().nanoseconds) * 1e-9

if __name__ == '__main__':
    rclpy.init()
    node = BrainProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()