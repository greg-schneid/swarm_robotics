Install the appropriate version of liblsl from below:
https://github.com/sccn/liblsl/releases


```bash
pip install muselsl
pip install -U bleak muselsl
```

# How to Run

1. Start the stream of data

Command to stream data -- do this in one terminal

```bash
muselsl stream --backend bleak
```

2. Once a connection is established, launch the ros2 nodes:
```bash
ros2 launch brain_streamer brain_system.launch.py
```

3. Check out raw data stream
``` bash
ros2 topic echo /brain.state
```

4. launch the data_streamer!!
``` bash
ros2 run data_streamer data_streamer --ros-args -p dest_host:=192.168.10.15 -p dest_port:=5005 -p broadcast:=true
```

5. make sure all nodes are present (i.e. brain_processor + brain_stream + data_streamer are all present)
``` bash
ros2 node list
```

       # Optional: do a quick, blocking calibration at startup
        self.get_logger().info('Calibration starting…')
        time.sleep(2.0)  # Pre-fill buffer ~2 s before we calibrate
        try:~
            self.processor.calibrate(relaxed_sec=10.0, focus_sec=10.0, interactive=False, logger=self.get_logger().info)
            self.get_logger().info('Calibration complete.')
        except RuntimeError as exc:
            self.get_logger().warning(f'Calibration skipped: {exc}')

        # Start streaming in a background thread
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        # ROS timers for processing and summaries
        self.create_timer(1.0 / max(self.process_frequency, 1e-6), self._process_tick)
        self.create_timer(self.summary_period, self.publish_data)

    def _on_action(self, action_msg: BrainActionMsg):
        # Publish the action message
        self.action_pub.publish(action_msg)

        self.get_logger().info(
            f"Action: {action_msg.action} | R={action_msg.beta_alpha_ratio:.2f} "
            f"α={action_msg.alpha_power:.2f} β={action_msg.beta_power:.2f}"
        )