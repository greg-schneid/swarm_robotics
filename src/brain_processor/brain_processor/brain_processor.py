#!/usr/bin/env python3
"""
brain-processor.py (rewritten)

- Subscribes: /brain/raw (swarm_messages/BrainData)
- Publishes:  /brain/state (std_msgs/Float32) in [0,1], 0=relaxed, 1=focused
- Optional:   /brain/metrics (Float32MultiArray): [state_raw, state, feature, alpha_dB, beta_dB, r_relax, r_focus]

Feature:
  feature = beta_rel_dB(frontal AF7/AF8) - alpha_rel_dB(temporal TP9/TP10)
  where rel_dB = 10*log10( band_power / broadband_power ), broadband = 1..45 Hz

Calibration:
  - Two phases (RELAXED, FOCUSED)
  - Collect short windows (1.0 s, step 0.25 s)
  - Use medians as anchors (r_relax, r_focus)
  - If r_focus < r_relax, swap them (ensures positive spread)
  - Enforce a minimum spread

Mapping:
  state_raw = clamp((feature - r_relax) / max(r_focus - r_relax, min_spread), 0, 1)
  state = EMA(state_raw, tau = state_smooth_tau)
"""

from __future__ import annotations
import threading, time
from typing import List, Optional, Tuple

import numpy as np
from scipy.signal import iirnotch, butter, filtfilt, welch, detrend

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
from swarm_messages.msg import BrainData


# ----------------------------- DSP helpers -----------------------------

def _bandpass(sig: np.ndarray, fs: float, lo: float, hi: float, order: int = 4) -> np.ndarray:
    b, a = butter(order, [lo/(fs/2), hi/(fs/2)], btype='band')
    # detrend before filtfilt helps with slow drifts
    return filtfilt(b, a, detrend(sig, type='linear'))

def _notch(sig: np.ndarray, fs: float, mains: float = 60.0, Q: float = 30.0) -> np.ndarray:
    b, a = iirnotch(w0=mains/(fs/2), Q=Q)
    return filtfilt(b, a, sig)

def _bandpower(sig: np.ndarray, fs: float, lo: float, hi: float, nperseg: int) -> float:
    # Welch with a bounded nperseg to avoid warnings and be robust to short windows
    nps = max(32, min(nperseg, len(sig)))
    f, pxx = welch(sig, fs=fs, nperseg=nps, noverlap=nps // 2, window="hann")
    mask = (f >= lo) & (f < hi)
    if not np.any(mask):
        return 0.0
    return float(np.trapz(pxx[mask], f[mask]))

def _safe_db(x: float) -> float:
    return 10.0 * np.log10(max(x, 1e-12))


# ----------------------------- Node -----------------------------

class BrainProcessor(Node):
    def __init__(self):
        super().__init__('brain_processor')

        # ---- Parameters ----
        self.declare_parameter('fs', 256.0)
        self.declare_parameter('buffer_sec', 4.0)
        self.declare_parameter('tick_hz', 5.0)
        self.declare_parameter('mains_hz', 60.0)

        self.declare_parameter('calib_relaxed_sec', 10.0)
        self.declare_parameter('calib_focus_sec', 10.0)

        self.declare_parameter('state_smooth_tau', 0.8)   # seconds
        self.declare_parameter('min_spread', 1.0)         # dB units, safer than 0.1 for stability

        self.declare_parameter('adapt_anchors', True)
        self.declare_parameter('adapt_rate', 0.005)
        self.declare_parameter('edge_low', 0.15)
        self.declare_parameter('edge_high', 0.85)

        self.declare_parameter('publish_metrics', True)

        # ---- Load params ----
        self.fs = float(self.get_parameter('fs').value)
        self.buffer_sec = float(self.get_parameter('buffer_sec').value)
        self.tick_hz = float(self.get_parameter('tick_hz').value)
        self.mains_hz = float(self.get_parameter('mains_hz').value)

        self.calib_relaxed_sec = float(self.get_parameter('calib_relaxed_sec').value)
        self.calib_focus_sec   = float(self.get_parameter('calib_focus_sec').value)

        self.state_smooth_tau = float(self.get_parameter('state_smooth_tau').value)
        self.min_spread = float(self.get_parameter('min_spread').value)

        self.adapt_anchors = bool(self.get_parameter('adapt_anchors').value)
        self.adapt_rate = float(self.get_parameter('adapt_rate').value)
        self.edge_low = float(self.get_parameter('edge_low').value)
        self.edge_high = float(self.get_parameter('edge_high').value)

        self.publish_metrics = bool(self.get_parameter('publish_metrics').value)

        # ---- Channels / indexing ----
        self.labels: List[str] = ["TP9", "AF7", "AF8", "TP10", "AUX"]
        self.C = len(self.labels)

        # ---- Ring buffer ----
        self.S = int(max(1, round(self.buffer_sec * self.fs)))
        self._rb = np.zeros((self.S, self.C), dtype=np.float64)
        self._write = 0
        self._filled = 0
        self._last_data_time: Optional[float] = None
        self._data_stale_warned = False

        # ---- Bands ----
        self.alpha_band = (8.0, 13.0)
        self.beta_band  = (13.0, 30.0)
        self.broad_band = (1.0, 45.0)

        # ---- Anchors (dB feature) ----
        self.r_relax: Optional[float] = None
        self.r_focus: Optional[float] = None

        # ---- State smoothing ----
        self._state = 0.0
        self._last_tick_time = self._now()

        # ---- Calibration ----
        self._calibrating = False
        self._cal_thread: Optional[threading.Thread] = None
        self._cal_buffer_lock = threading.Lock()
        self._cal_buffer: List[np.ndarray] = []
        self._cal_requested = False
        self._calibrated_once = False

        # ---- ROS I/O ----
        self.state_pub = self.create_publisher(Float32, '/brain/state', 20)
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/brain/metrics', 10) if self.publish_metrics else None

        self.create_subscription(BrainData, '/brain/raw', self._on_raw, 200)
        self.create_subscription(Bool, '/brain/connected', self._on_connected, 10)

        self.create_timer(1.0 / self.tick_hz, self._process_tick)

        self.get_logger().info(
            f"BrainProcessor ready | fs={self.fs} Hz, buffer={self.buffer_sec}s, tick={self.tick_hz} Hz, mains={self.mains_hz} Hz"
        )

    # ------------------------- ROS callbacks -------------------------

    def _on_connected(self, msg: Bool) -> None:
        if msg.data and not self._calibrating and not self._calibrated_once and not self._cal_requested:
            self._cal_requested = True
            self.get_logger().info("Brain connected. Starting calibration…")
            self._cal_thread = threading.Thread(target=self._calibration_thread, daemon=True)
            self._cal_thread.start()

    def _on_raw(self, data: BrainData) -> None:
        v = np.asarray(data.values, dtype=np.float64).reshape(-1)
        if v.size != self.C:
            if v.size == self.C - 1:
                v = np.concatenate([v, [0.0]])  # tolerate missing AUX
            else:
                return
        self._push_row(v)
        self._last_data_time = self._now()
        self._data_stale_warned = False
        if self._calibrating:
            with self._cal_buffer_lock:
                self._cal_buffer.append(v.copy())

    # --------------------------- Processing ---------------------------

    def _process_tick(self) -> None:
        if self._filled < int(self.fs * 1.5):
            return

        # stale check
        if self._last_data_time is not None and (self._now() - self._last_data_time) > 2.0:
            if not self._data_stale_warned:
                self.get_logger().warning("EEG feed stale; skipping tick.")
                self._data_stale_warned = True
            return
        else:
            self._data_stale_warned = False

        alpha_db, beta_db, feature = self._compute_feature(self._window())

        # not calibrated yet: publish metrics only (optional)
        if self.r_relax is None or self.r_focus is None:
            if self.metrics_pub:
                m = Float32MultiArray()
                m.data = [float('nan'), float('nan'), float(feature), float(alpha_db), float(beta_db), float('nan'), float('nan')]
                self.metrics_pub.publish(m)
            return

        spread = max(self.r_focus - self.r_relax, self.min_spread)
        state_raw = (feature - self.r_relax) / spread
        state_raw = float(np.clip(state_raw, 0.0, 1.0))

        # EMA smoothing
        now = self._now()
        dt = max(1e-3, now - self._last_tick_time)
        self._last_tick_time = now
        tau = max(1e-3, self.state_smooth_tau)
        a = 1.0 - np.exp(-dt / tau)
        self._state = (1.0 - a) * self._state + a * state_raw

        # optional slow drift adaptation
        if self.adapt_anchors:
            if self._state < self.edge_low:
                self.r_relax = (1.0 - self.adapt_rate) * self.r_relax + self.adapt_rate * feature
            elif self._state > self.edge_high:
                self.r_focus = (1.0 - self.adapt_rate) * self.r_focus + self.adapt_rate * feature
            # keep minimum spread
            if (self.r_focus - self.r_relax) < self.min_spread:
                mid = 0.5 * (self.r_focus + self.r_relax)
                self.r_relax = mid - 0.5 * self.min_spread
                self.r_focus = mid + 0.5 * self.min_spread

        # publish
        s = Float32(); s.data = float(self._state)
        self.state_pub.publish(s)

        if self.metrics_pub:
            m = Float32MultiArray()
            m.data = [state_raw, self._state, float(feature), float(alpha_db), float(beta_db), float(self.r_relax), float(self.r_focus)]
            self.metrics_pub.publish(m)

    # --------------------------- Calibration --------------------------

    def _calibration_thread(self):
        try:
            self._calibrating = True

            self.get_logger().info("Calibration: Phase 1/2 RELAXED (eyes closed).")
            r_relax_list = self._collect_phase_features("RELAXED", self.calib_relaxed_sec, win_sec=1.0, step_sec=0.25)
            if len(r_relax_list) < 5:
                self.get_logger().error(f"Calibration failed: only {len(r_relax_list)} relaxed windows.")
                return

            self.get_logger().info("Calibration: Phase 2/2 FOCUSED (mental math).")
            r_focus_list = self._collect_phase_features("FOCUSED", self.calib_focus_sec, win_sec=1.0, step_sec=0.25)
            if len(r_focus_list) < 5:
                self.get_logger().error(f"Calibration failed: only {len(r_focus_list)} focused windows.")
                return

            r_relax = float(np.median(r_relax_list))
            r_focus = float(np.median(r_focus_list))

            # Ensure focused anchor is above relaxed; swap if inverted
            if r_focus < r_relax:
                self.get_logger().warn(f"Anchors inverted (focus < relax); swapping. relax={r_relax:.2f} focus={r_focus:.2f}")
                r_relax, r_focus = r_focus, r_relax

            # Enforce minimum spread
            if (r_focus - r_relax) < self.min_spread:
                mid = 0.5 * (r_focus + r_relax)
                r_relax = mid - 0.5 * self.min_spread
                r_focus = mid + 0.5 * self.min_spread

            self.r_relax = r_relax
            self.r_focus = r_focus
            self._state = 0.0
            self._calibrated_once = True

            self.get_logger().info(
                f"Calibration done: r_relax≈{self.r_relax:.2f} dB, r_focus≈{self.r_focus:.2f} dB, "
                f"spread≈{(self.r_focus - self.r_relax):.2f} dB"
            )
        finally:
            self._calibrating = False
            self._cal_requested = False

    def _collect_phase_features(self, name: str, duration: float, win_sec: float, step_sec: float) -> List[float]:
        with self._cal_buffer_lock:
            self._cal_buffer.clear()

        self.get_logger().info(f"Phase {name}: starting in 1s…")
        time.sleep(1.0)

        t0 = time.time()
        while (time.time() - t0) < duration and rclpy.ok():
            time.sleep(0.01)  # allow subscriber to fill buffer

        with self._cal_buffer_lock:
            cal = np.array(self._cal_buffer, dtype=np.float64)

        feats: List[float] = []
        if cal.shape[0] == 0:
            return feats

        w = int(max(1, round(win_sec * self.fs)))
        s = int(max(1, round(step_sec * self.fs)))
        for i in range(0, cal.shape[0] - w + 1, s):
            win = cal[i:i+w, :]
            _, _, f = self._compute_feature(win)
            feats.append(float(f))

        self.get_logger().info(f"Phase {name}: computed {len(feats)} windows.")
        return feats

    # ------------------------- Internals -------------------------

    def _idx(self, name: str) -> int:
        mapping = {n: i for i, n in enumerate(self.labels)}
        defaults = {"TP9": 0, "AF7": 1, "AF8": 2, "TP10": 3, "AUX": 4}
        return mapping.get(name, defaults[name])

    def _compute_feature(self, win: np.ndarray) -> Tuple[float, float, float]:
        """Return (alpha_rel_dB_temporal, beta_rel_dB_frontal, feature_dB)."""
        fs = self.fs

        # Filter channel-wise
        filt = np.empty_like(win)
        for c in range(win.shape[1]):
            x = win[:, c]
            x = _bandpass(x, fs, 1.0, 45.0)
            x = _notch(x, fs, self.mains_hz)
            filt[:, c] = x

        # channel groups
        tp9  = filt[:, self._idx("TP9")]
        tp10 = filt[:, self._idx("TP10")]
        af7  = filt[:, self._idx("AF7")]
        af8  = filt[:, self._idx("AF8")]

        # choose an nperseg that fits short windows well
        nps = int(min(max(64, fs), len(tp9)))  # ~1.0 s at 64–256

        # Temporal alpha (relative to broadband) in dB
        alpha_tp9  = _bandpower(tp9,  fs, *self.alpha_band, nps)
        alpha_tp10 = _bandpower(tp10, fs, *self.alpha_band, nps)
        broad_tp9  = _bandpower(tp9,  fs, *self.broad_band, nps)
        broad_tp10 = _bandpower(tp10, fs, *self.broad_band, nps)

        alpha_rel_temporal = 0.5 * ((alpha_tp9 / (broad_tp9 + 1e-12)) +
                                    (alpha_tp10 / (broad_tp10 + 1e-12)))
        alpha_db_temporal = _safe_db(alpha_rel_temporal)

        # Frontal beta (relative) in dB
        beta_af7  = _bandpower(af7, fs, *self.beta_band, nps)
        beta_af8  = _bandpower(af8, fs, *self.beta_band, nps)
        broad_af7 = _bandpower(af7, fs, *self.broad_band, nps)
        broad_af8 = _bandpower(af8, fs, *self.broad_band, nps)

        beta_rel_frontal = 0.5 * ((beta_af7 / (broad_af7 + 1e-12)) +
                                  (beta_af8 / (broad_af8 + 1e-12)))
        beta_db_frontal = _safe_db(beta_rel_frontal)

        feature = beta_db_frontal - alpha_db_temporal
        return alpha_db_temporal, beta_db_frontal, feature

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
