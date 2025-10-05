import rclpy
from rclpy.node import Node

from pylsl import StreamInlet, resolve_streams
import subprocess
import time
import numpy as np
from queue import Queue
from typing import Optional, List
from concurrency import MultiThread, run_async
from brain_processor import BrainProcessor, BrainData, BrainActionMsg

class BrainStream(Node):
    def __init__(self, process_frequency: int = 5, mains_hz: float = 60.0):
        super().__init__()
        self.receive_frequency = 256  # Hz
        self.publish_frequency = process_frequency  # Hz
        self.data_queue: "Queue[BrainData]" = Queue(maxsize=10000)  # Rolling queue for storing brain data
        
        # LSL stream objects
        self.stream = None
        self.inlet: Optional[StreamInlet] = None
        self.labels: List[str] = ["TP9", "AF7", "AF8", "TP10", "AUX"]
        
        # Start muselsl stream in background
        self._get_stream()

        self.processor = BrainProcessor(
            sample_rate=self.receive_frequency,
            labels=self.labels,
            mains_hz=mains_hz,
            action_callback=self._on_action
        )

        # Optional: do a quick, blocking calibration at startup
        print("\n[BrainStream] Calibration starting…")
        print("  Phase 1: RELAXED (eyes closed) for ~10 s")
        print("  Phase 2: FOCUS (mental math) for ~10 s\n")
        # Pre-fill buffer ~2 s before we calibrate
        time.sleep(2.0)
        self.processor.calibrate(relaxed_sec=10.0, focus_sec=10.0)
        print("[BrainStream] Calibration done.\n")

        self.stream_data()  # Start streaming data
        self.timer(5, self.publish_data)

    def _on_action(self, BrainActionMsg):
        print(f"[BrainStream] Action: {BrainActionMsg.action}, "
              f"Beta/Alpha Ratio: {BrainActionMsg.beta_alpha_ratio:.2f}, "
              f"Alpha Power: {BrainActionMsg.alpha_power:.2f}, "
              f"Beta Power: {BrainActionMsg.beta_power:.2f}")

    def _start_stream(self):
        print("Starting muselsl stream...")
        self.muselsl_process = subprocess.Popen(
            ['muselsl', 'stream', '--backend', 'bleak'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # Give it a moment to start up
        time.sleep(2)
        print("muselsl stream started.")

    def _get_stream(self):
        print("Getting LSL streams...")
        streams = resolve_streams(wait_time=5.0)
        if not streams:
            self.stream = None
            raise RuntimeError("No LSL EEG stream found. Is muselsl still running?")
        print(f"Found {len(streams)} streams.")
        for stream in streams:
            print(f"Stream Name: {stream.name()}, Type: {stream.type()}, ID: {stream.source_id()}")
            if stream.name() == 'Muse' and stream.type() == 'EEG':
                self.stream = stream
                self.receive_frequency = stream.nominal_srate()
                self.inlet = StreamInlet(self.stream)
                print(f"Found stream from EEG")
                return
        self.stream = None
        raise RuntimeError("Unable to Find Muse stream. Is Muse still running?")        

    def stop(self):
        self.shutdown_event.set()
        time.sleep(0.05)
    
    def _stop_stream(self):
        # Close the subprocess when the object is deleted
        if hasattr(self, 'muselsl_process') and self.muselsl_process:
            print("Stopping muselsl stream...")
            self.muselsl_process.terminate()
            try:
                self.muselsl_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.muselsl_process.kill()
            print("muselsl stream stopped.")

    def print_stream_properties(self):
        if not self.stream:
            print("No stream available.")
            return
        s = self.stream
        print(f"Stream Name: {s.name()}, Type: {s.type()}, ID: {s.source_id()}")
        print(f"  Channels: {s.channel_count()}  fs: {s.nominal_srate()} Hz  format: {s.channel_format()}")
        print(f"  UID: {s.uid()}  Host: {s.hostname()}  Session: {s.session_id()}")

    @run_async
    def stream_data(self):
        if not self.inlet:
            raise RuntimeError("Stream inlet not initialized.")
            
        while not self.shutdown_event.is_set():
            sample, ts = self.inlet.pull_sample(timeout=1.0)
            if sample is None:
                continue

            data = BrainData(timestamp=ts, values=np.array(sample))
            self.processor.receive_data(data)
            self.data_queue.put(data)


    def publish_data(self):
        """Calculate and print average of brain data from the last 5 seconds."""
        current_time = time.time()
        cutoff_time = current_time - 5.0  # 5 seconds ago
        
        # Collect all data from queue and filter for last 5 seconds
        recent_data = []
        temp_items = []
        
        # Empty the queue and collect items
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                temp_items.append(data)
                # Keep data from last 5 seconds
                if data.timestamp >= cutoff_time:
                    recent_data.append(data.values)
            except:
                break
        
        # Put back only data from last 5 seconds (rolling window)
        for item in temp_items:
            if item.timestamp >= cutoff_time:
                self.data_queue.put(item)
        
        # Calculate and print average
        if recent_data:
            average_values = np.mean(recent_data, axis=0)
            print(f"Average over last 5 seconds ({len(recent_data)} samples):")
            for i, avg in enumerate(average_values):
                print(f"  Channel {i}: {avg:.4f}")
        else:
            print("No data available in the last 5 seconds.")

    def sample_data(self, num_samples=10, timeout=2.0):
        if not hasattr(self, 'inlet'):
            raise RuntimeError("Stream inlet not initialized.")
        samples = []
        timestamps = []
        for _ in range(num_samples):
            sample, ts = self.inlet.pull_sample(timeout=timeout)
            if sample:
                samples.append(sample)
                timestamps.append(ts)
        return np.array(samples), np.array(timestamps)

    def _process_tick(self):
        self.processor.process_tick()

    def __del__(self):
        self.stop()

if __name__ == "__main__":
    brain_stream = BrainStream()
    brain_stream.print_stream_properties()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
