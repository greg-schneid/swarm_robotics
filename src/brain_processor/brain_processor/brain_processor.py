#!/usr/bin/env python3
"""
brain-processor.py

ROS2 node that subscribes to /brain/raw (swarm_messages/BrainData),
computes a continuous mental-state value in [0,1] where:
    0.0 = relaxed (alpha-dominant)
    1.0 = focused (beta-dominant)
and publishes it on /brain/state (std_msgs/Float32).

Key steps:
- 4 s ring buffer of EEG (TP9, AF7, AF8, TP10, AUX)
- Band-pass 1–45 Hz + Notch (50/60 Hz)
- Welch PSD → Alpha power (8–13 Hz) from TP9/TP10, Beta power (13–30 Hz) from AF7/AF8
- Ratio r = Beta / Alpha
- Calibration learns relaxed and focused anchors (r_relax, r_focus)
- Normalize: state_raw = clamp((r - r_relax) / (r_focus - r_relax), 0, 1)
- Smooth with exponential filter; optional slow drift adaptation of anchors
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional, Tuple

import numpy as np
from scipy.signal import iirnotch, butter, filtfilt, welch, detrend

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool

# Custom message from your package
from swarm_messages.msg import BrainData


# ----------------------------- DSP helpers -----------------------------

def _bandpass(sig: np.ndarray, fs: float, lo: float, hi: float, order: int = 4) -> np.ndarray:
    b, a = butter(order, [lo/(fs/2), hi/(fs/2)], btype='band')
    return filtfilt(b, a, detrend(sig, type='linear'))

def _notch(sig: np.ndarray, fs: float, mains: float = 60.0, Q: float = 30.0) -> np.ndarray:
    b, a = iirnotch(w0=mains/(fs/2), Q=Q)
    return filtfilt(b, a, sig)

def _bandpower_welch(sig: np.ndarray, fs: float, f_lo: float, f_hi: float) -> float:
    nps = max(64, int(fs * 2))  # ~2 s segments for decent resolution
    f, pxx = welch(sig, fs=fs, nperseg=nps, noverlap=nps // 2, window="hann")
    mask = (f >= f_lo) & (f < f_hi)
    if not np.any(mask):
        return 0.0
    return float(np.trapz(pxx[mask], f[mask]))


# ----------------------------- Node -----------------------------

class BrainProcessor(Node):
    """
    Continuous mental-state estimator:
      - Keeps a ring buffer window of EEG
      - Filters + PSD
      - Beta/Alpha ratio → normalized to [0,1] via calibrated anchors
      - Smoothed and published on /brain/state
    """

    def __init__(self):
        super().__init__('brain_processor')

        # ---------- Parameters (tunable) ----------
        self.declare_parameter('fs', 256.0)                 # Muse nominal sample rate
        self.declare_parameter('buffer_sec', 4.0)           # ring buffer seconds
        self.declare_parameter('tick_hz', 5.0)              # processing cadence
        self.declare_parameter('mains_hz', 60.0)            # set 50.0 if applicable

        # Calibration durations (s)
        self.declare_parameter('calib_relaxed_sec', 10.0)
        self.declare_parameter('calib_focus_sec', 10.0)
        # Smoothing time constant (s) for the final state
        self.declare_parameter('state_smooth_tau', 0.8)
        # Optional slow adaptation of anchors (EMA) when near edges
        self.declare_parameter('adapt_anchors', True)
        self.declare_parameter('adapt_rate', 0.005)         # small EMA step
        self.declare_parameter('edge_low', 0.15)            # when state < edge_low, adapt relaxed anchor
        self.declare_parameter('edge_high', 0.85)           # when state > edge_high, adapt focus anchor
        # Minimum spread to avoid divide-by-near-zero (absolute, ratio units)
        self.declare_parameter('min_spread', 0.1)

        # Debug metrics publisher toggle
        self.declare_parameter('publish_metrics', True)

        # ---------- Load params ----------
        self.fs = float(self.get_parameter('fs').value)
        self.buffer_sec = float(self.get_parameter('buffer_sec').value)
        self.tick_hz = float(self.get_parameter('tick_hz').value)
        self.mains_hz = float(self.get_parameter('mains_hz').value)

        self.calib_relaxed_sec = float(self.get_parameter('calib_relaxed_sec').value)
        self.calib_focus_sec = float(self.get_parameter('calib_focus_sec').value)

        self.state_smooth_tau = float(self.get_parameter('state_smooth_tau').value)
        self.adapt_anchors = bool(self.get_parameter('adapt_anchors').value)
        self.adapt_rate = float(self.get_parameter('adapt_rate').value)
        self.edge_low = float(self.get_parameter('edge_low').value)
        self.edge_high = float(self.get_parameter('edge_high').value)
        self.min_spread = float(self.get_parameter('min_spread').value)

        self.publish_metrics = bool(self.get_parameter('publish_metrics').value)

        # ---------- Channel labels ----------
        self.labels: List[str] = ["TP9", "AF7", "AF8", "TP10", "AUX"]
        self.C = len(self.labels)

        # ---------- Ring buffer ----------
        self.S = int(max(1, round(self.buffer_sec * self.fs)))
        self._rb = np.zeros((self.S, self.C), dtype=np.float64)
        self._write = 0
        self._filled = 0
        self._last_data_time: Optional[float] = None
        self._data_stale_warned = False

        # ---------- Bands ----------
        self.alpha_band = (8.0, 13.0)
        self.beta_band = (13.0, 30.0)

        # ---------- Anchors learned by calibration ----------
        self.r_relax: Optional[float] = None
        self.r_focus: Optional[float] = None

        # ---------- State smoothing ----------
        self._state_smoothed: float = 0.0  # starts at relaxed
        self._last_tick_time = self._now()

        # ---------- Calibration machinery ----------
        self._calibrating = False
        self._cal_thread: Optional[threading.Thread] = None
        self._cal_buffer_lock = threading.Lock()
        self._cal_buffer: List[np.ndarray] = []
        self._cal_requested = False
        self._calibrated_once = False

        # ---------- ROS I/O ----------
        self.state_pub = self.create_publisher(Float32, '/brain/state', 20)
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/brain/metrics', 10) if self.publish_metrics else None

        # Subscriptions
        self.create_subscription(BrainData, '/brain/raw', self._on_raw, 200)
        self.create_subscription(Bool, '/brain/connected', self._on_connected, 10)

        # Processing timer
        self.create_timer(1.0 / self.tick_hz, self._process_tick)

        self.get_logger().info(
            f"BrainProcessor up | fs={self.fs} Hz, buffer={self.buffer_sec}s, "
            f"tick={self.tick_hz} Hz, mains={self.mains_hz} Hz"
        )

    # ------------------------- ROS callbacks -------------------------

    def _on_connected(self, msg: Bool) -> None:
        if msg.data and not self._calibrating and not self._calibrated_once and not self._cal_requested:
            self._cal_requested = True
            self.get_logger().info("Brain connected. Starting calibration thread…")
            self._cal_thread = threading.Thread(target=self._calibration_thread, daemon=True)
            self._cal_thread.start()

    def _on_raw(self, data: BrainData) -> None:
        v = np.asarray(data.values, dtype=np.float64).reshape(-1)
        if v.size != self.C:
            # tolerate missing AUX (C-1)
            if v.size == self.C - 1:
                v = np.concatenate([v, [0.0]])
            else:
                return
        self._push_row(v)
        self._last_data_time = self._now()
        self._data_stale_warned = False

        # If we are in a calibration phase, stash the sample
        if self._calibrating:
            with self._cal_buffer_lock:
                self._cal_buffer.append(v.copy())

    # --------------------------- Processing ---------------------------

    def _process_tick(self) -> None:
        # Need at least ~1.5 s of data to do anything
        if self._filled < int(self.fs * 1.5):
            return

        # Guard against stale input
        if self._last_data_time is not None:
            stale = self._now() - self._last_data_time
            if stale > 2.0:
                if not self._data_stale_warned:
                    self.get_logger().warning(f"EEG feed stale for {stale:.1f}s; skipping.")
                    self._data_stale_warned = True
                return
            else:
                self._data_stale_warned = False

        # Compute features on the current window
        alpha, beta, ratio = self._summarize_ratio(self._window())

        # If not calibrated yet, publish nothing (or publish raw ratio as metric)
        if self.r_relax is None or self.r_focus is None:
            if self.metrics_pub:
                m = Float32MultiArray()
                m.data = [float('nan'), float('nan'), float(ratio), float(alpha), float(beta)]
                self.metrics_pub.publish(m)
            return

        # Normalize ratio → [0,1]
        spread = max(self.r_focus - self.r_relax, self.min_spread)
        state_raw = (ratio - self.r_relax) / spread
        state_raw = float(np.clip(state_raw, 0.0, 1.0))

        # Smooth
        now = self._now()
        dt = max(1e-3, now - self._last_tick_time)
        self._last_tick_time = now
        # exponential smoothing coefficient from time constant tau
        tau = max(1e-3, self.state_smooth_tau)
        alpha_s = 1.0 - np.exp(-dt / tau)
        self._state_smoothed = (1.0 - alpha_s) * self._state_smoothed + alpha_s * state_raw

        # Optional slow adaptation of anchors (drift handling)
        if self.adapt_anchors:
            if self._state_smoothed < self.edge_low:
                # drift relaxed anchor toward current ratio very slowly
                self.r_relax = (1.0 - self.adapt_rate) * self.r_relax + self.adapt_rate * ratio
            elif self._state_smoothed > self.edge_high:
                # drift focused anchor toward current ratio
                self.r_focus = (1.0 - self.adapt_rate) * self.r_focus + self.adapt_rate * ratio
            # maintain a minimum spread
            if (self.r_focus - self.r_relax) < self.min_spread:
                mid = 0.5 * (self.r_focus + self.r_relax)
                self.r_relax = mid - 0.5 * self.min_spread
                self.r_focus = mid + 0.5 * self.min_spread

        # Publish state
        out = Float32()
        out.data = float(self._state_smoothed)
        self.state_pub.publish(out)

        # Optional metrics: [state_raw, state_smoothed, ratio, alpha, beta]
        if self.metrics_pub:
            m = Float32MultiArray()
            m.data = [state_raw, self._state_smoothed, float(ratio), float(alpha), float(beta)]
            self.metrics_pub.publish(m)

    # --------------------------- Calibration --------------------------

    def _calibration_thread(self):
        """Two-phase calibration running off the executor thread."""
        try:
            self._calibrating = True
            self.get_logger().info("Calibration: Phase 1/2 RELAXED (eyes closed).")
            r_relax_samples = self._collect_phase_ratios(
                phase_name="RELAXED",
                duration=self.calib_relaxed_sec,
                win_sec=2.0,
                step_sec=0.5
            )
            if len(r_relax_samples) < 5:
                self.get_logger().error(
                    f"Calibration failed: only {len(r_relax_samples)} relaxed windows."
                )
                self._calibrating = False
                self._cal_requested = False
                return

            self.get_logger().info("Calibration: Phase 2/2 FOCUSED (mental math).")
            r_focus_samples = self._collect_phase_ratios(
                phase_name="FOCUSED",
                duration=self.calib_focus_sec,
                win_sec=2.0,
                step_sec=0.5
            )
            if len(r_focus_samples) < 5:
                self.get_logger().error(
                    f"Calibration failed: only {len(r_focus_samples)} focused windows."
                )
                self._calibrating = False
                self._cal_requested = False
                return

            # Robust anchors: use medians
            r_relax = float(np.median(r_relax_samples))
            r_focus = float(np.median(r_focus_samples))

            # Ensure a reasonable spread
            spread = r_focus - r_relax
            if spread < self.min_spread:
                self.get_logger().warn(
                    f"Calibration spread too small ({spread:.3f}); enforcing min_spread={self.min_spread:.3f}."
                )
                mid = 0.5 * (r_focus + r_relax)
                r_relax = mid - 0.5 * self.min_spread
                r_focus = mid + 0.5 * self.min_spread

            self.r_relax = r_relax
            self.r_focus = r_focus
            self._calibrated_once = True

            # Set initial smoothed state near relaxed
            self._state_smoothed = 0.0

            self.get_logger().info(
                f"Calibration done: r_relax≈{self.r_relax:.2f}, r_focus≈{self.r_focus:.2f}, "
                f"spread≈{(self.r_focus - self.r_relax):.2f}"
            )
        finally:
            self._calibrating = False
            self._cal_requested = False

    def _collect_phase_ratios(
        self,
        phase_name: str,
        duration: float,
        win_sec: float,
        step_sec: float
    ) -> List[float]:
        """Collect Beta/Alpha ratios for a phase by recording raw samples, then sliding-window PSD."""
        # clear buffer for this phase
        with self._cal_buffer_lock:
            self._cal_buffer.clear()

        # announce and give users a brief second to settle
        self.get_logger().info(f"Phase {phase_name}: starting in 1s…")
        time.sleep(1.0)

        # collect for 'duration' seconds
        t0 = time.time()
        while (time.time() - t0) < duration and rclpy.ok():
            time.sleep(0.01)  # allow subscriber to fill _cal_buffer

        # snapshot
        with self._cal_buffer_lock:
            cal = np.array(self._cal_buffer, dtype=np.float64)

        if cal.shape[0] < int(0.5 * duration * self.fs):
            self.get_logger().warn(
                f"Phase {phase_name}: low sample count: {cal.shape[0]}."
            )

        ratios: List[float] = []
        if cal.shape[0] == 0:
            return ratios

        w = int(max(1, round(win_sec * self.fs)))
        s = int(max(1, round(step_sec * self.fs)))
        for i in range(0, cal.shape[0] - w + 1, s):
            window = cal[i:i+w, :]
            _, _, r = self._summarize_ratio(window)
            ratios.append(float(r))

        self.get_logger().info(f"Phase {phase_name}: computed {len(ratios)} ratio windows.")
        return ratios

    # ------------------------- Internals -------------------------

    def _idx(self, name: str) -> int:
        mapping = {n: i for i, n in enumerate(self.labels)}
        defaults = {"TP9": 0, "AF7": 1, "AF8": 2, "TP10": 3, "AUX": 4}
        return mapping.get(name, defaults[name])

    def _summarize_ratio(self, win: np.ndarray) -> Tuple[float, float, float]:
        fs = self.fs
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

        alpha = 0.5 * (_bandpower_welch(tp9, fs, *self.alpha_band) +
                       _bandpower_welch(tp10, fs, *self.alpha_band))
        beta  = 0.5 * (_bandpower_welch(af7, fs, *self.beta_band)  +
                       _bandpower_welch(af8, fs, *self.beta_band))
        ratio = beta / (alpha + 1e-9)
        return alpha, beta, ratio

    def _push_row(self, row: np.ndarray) -> None:
        self._rb[self._write, :] = row
        self._write = (self._write + 1) % self.S
        self._filled = min(self.S, self._filled + 1)

    def _window(self) -> np.ndarray:
        if self._filled < self.S:
            return self._rb[:self._filled, :]
        if self._write == 0:
            return self._rb
        return np.vstack((self._rb[self._write:, :], self._rb[:self._write, :]))

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9


def main(args=None):
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
