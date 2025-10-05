import threading
import time
from queue import Queue
from typing import List, Optional

import numpy as np
import rclpy
from pylsl import StreamInlet, resolve_streams
from rclpy.node import Node
import subprocess
from std_msgs.msg import Float32MultiArray

from swarm_messages.msg import BrainData, BrainActionMsg

class BrainStream(Node):
    def __init__(self, process_frequency: int = 5, mains_hz: float = 60.0, summary_period: float = 5.0):
        super().__init__('brain_stream')
        self.receive_frequency = 256  # Hz
        self.process_frequency = int(process_frequency)
        self.summary_period = summary_period
        self.data_queue: "Queue[BrainData]" = Queue(maxsize=10000)
        self.shutdown_event = threading.Event()

        # Publishers for raw data and processed outputs
        self.raw_pub = self.create_publisher(Float32MultiArray, '/brain/raw', 50)
        self.average_pub = self.create_publisher(Float32MultiArray, 'brain/raw_average', 10)

        # LSL stream objects
        self.stream = None
        self.inlet: Optional[StreamInlet] = None
        self.labels: List[str] = ["TP9", "AF7", "AF8", "TP10", "AUX"]

        self.get_logger().info('Resolving LSL stream…')
        self._get_stream()

        # Start streaming in a background thread
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        # ROS timer for summaries
        self.create_timer(self.summary_period, self.publish_data)

    def _start_stream(self):
        self.get_logger().info('Starting muselsl stream…')
        self.muselsl_process = subprocess.Popen(
            ['muselsl', 'stream', '--backend', 'bleak'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # Give it a moment to start up
        time.sleep(2)
        self.get_logger().info('muselsl stream started.')

    def _get_stream(self):
        self.get_logger().info('Looking for Muse EEG stream…')
        streams = resolve_streams(wait_time=5.0)
        if not streams:
            self.stream = None
            raise RuntimeError("No LSL EEG stream found. Is muselsl still running?")
        self.get_logger().info(f'Found {len(streams)} LSL streams.')
        for stream in streams:
            self.get_logger().debug(
                f"Stream Name: {stream.name()}, Type: {stream.type()}, ID: {stream.source_id()}"
            )
            if stream.name() == 'Muse' and stream.type() == 'EEG':
                self.stream = stream
                self.receive_frequency = stream.nominal_srate()
                self.inlet = StreamInlet(self.stream)
                self.get_logger().info('Connected to Muse EEG stream.')
                return
        self.stream = None
        raise RuntimeError("Unable to Find Muse stream. Is Muse still running?")        

    def stop(self):
        self.shutdown_event.set()
        time.sleep(0.05)
    
    def _stop_stream(self):
        # Close the subprocess when the object is deleted
        if hasattr(self, 'muselsl_process') and self.muselsl_process:
            self.get_logger().info('Stopping muselsl stream…')
            self.muselsl_process.terminate()
            try:
                self.muselsl_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.muselsl_process.kill()
            self.get_logger().info('muselsl stream stopped.')

    def print_stream_properties(self):
        if not self.stream:
            self.get_logger().warning('No stream available.')
            return
        s = self.stream
        self.get_logger().info(
            f"Stream Name: {s.name()}, Type: {s.type()}, ID: {s.source_id()}\n"
            f"  Channels: {s.channel_count()}  fs: {s.nominal_srate()} Hz  format: {s.channel_format()}\n"
            f"  UID: {s.uid()}  Host: {s.hostname()}  Session: {s.session_id()}"
        )

    def _stream_loop(self):
        if not self.inlet:
            raise RuntimeError("Stream inlet not initialized.")

        while rclpy.ok():
            sample, ts = self.inlet.pull_sample(timeout=1.0)
            if sample is None or ts is None:
                continue

            # Create ROS2 BrainData message
            data = BrainData()
            data.timestamp = float(ts)
            data.values = [float(v) for v in sample]

            try:
                self.data_queue.put_nowait(data)
            except Exception:
                # queue full – drop the oldest item to maintain window
                try:
                    self.data_queue.get_nowait()
                except Exception:
                    pass
                self.data_queue.put_nowait(data)

            # Publish ROS2 BrainData message
            self.raw_pub.publish(data)


    def publish_data(self):
        """Calculate and print average of brain data from the last 5 seconds."""
        current_time = time.time()
        cutoff_time = current_time - self.process_frequency  # process_frequency seconds ago
        
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
                    # values is a list in ROS2 message
                    recent_data.append(np.array(data.values))
            except:
                break
        
        # Put back only data from last 5 seconds (rolling window)
        for item in temp_items:
            if item.timestamp >= cutoff_time:
                self.data_queue.put(item)
        
        # Calculate and print average
        if recent_data:
            average_values = np.mean(recent_data, axis=0)
            average_msg = Float32MultiArray()
            average_msg.data = [float(v) for v in average_values.tolist()]
            self.average_pub.publish(average_msg)
            self.get_logger().debug(
                f"Average over last 5 seconds ({len(recent_data)} samples): {[round(v, 4) for v in average_values]}"
            )
        else:
            self.get_logger().debug('No data available in the last 5 seconds.')

    def sample_data(self, num_samples=10, timeout=2.0):
        if not self.inlet:
            raise RuntimeError("Stream inlet not initialized.")
        samples = []
        timestamps = []
        for _ in range(num_samples):
            sample, ts = self.inlet.pull_sample(timeout=timeout)
            if sample:
                samples.append(sample)
                timestamps.append(ts)
        return np.array(samples), np.array(timestamps)
    
    def __del__(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = BrainStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
