#!/usr/bin/env python3
"""
data_streamer.py

Subscribes to /brain/state (std_msgs/Float32) and sends each value via UDP.

Parameters:
  dest_host (string)          : destination hostname or IP (default: "127.0.0.1")
  dest_port (int)             : destination UDP port (default: 5005)
  udp_format (string)         : "text" | "json" | "binary" (default: "text")
  include_timestamp (bool)    : include UNIX seconds in "text"/"json" or pack a double before the float in "binary" (default: false)
  precision (int)             : decimal places for "text" format (default: 6)
  broadcast (bool)            : enable UDP broadcast (sets SO_BROADCAST) (default: false)

Topic:
  /brain/state : std_msgs/Float32 (value in [0,1])
"""

from __future__ import annotations

import json
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class BrainUDPStreamer(Node):
    def __init__(self):
        super().__init__("data_streamer")

        # Declare parameters with sensible defaults
        self.declare_parameter("dest_host", "127.0.0.1")
        self.declare_parameter("dest_port", 5005)
        self.declare_parameter("udp_format", "text")  # "text" | "json" | "binary"
        self.declare_parameter("include_timestamp", False)
        self.declare_parameter("precision", 6)
        self.declare_parameter("broadcast", False)

        # Load parameters
        self.dest_host: str = self.get_parameter("dest_host").value
        self.dest_port: int = int(self.get_parameter("dest_port").value)
        self.udp_format: str = str(self.get_parameter("udp_format").value).lower()
        self.include_ts: bool = bool(self.get_parameter("include_timestamp").value)
        self.precision: int = int(self.get_parameter("precision").value)
        self.broadcast: bool = bool(self.get_parameter("broadcast").value)

        # Validate format
        if self.udp_format not in ("text", "json", "binary"):
            self.get_logger().warn(
                f"Unknown udp_format='{self.udp_format}', falling back to 'text'."
            )
            self.udp_format = "text"

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        if self.broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.dest = (self.dest_host, self.dest_port)
        self.get_logger().info(
            f"Streaming /brain/state via UDP to {self.dest_host}:{self.dest_port} "
            f"as {self.udp_format.upper()} (include_timestamp={self.include_ts}, broadcast={self.broadcast})"
        )

        # Subscribe to brain state
        self.sub = self.create_subscription(Float32, "/brain/state", self._on_state, 50)

        # Allow dynamic parameter updates at runtime
        self.add_on_set_parameters_callback(self._on_param_change)

    # ---- ROS Callbacks ----

    def _on_state(self, msg: Float32) -> None:
        value = float(msg.data)
        t = time.time()

        try:
            if self.udp_format == "text":
                if self.include_ts:
                    payload = f"{t:.6f},{value:.{self.precision}f}\n".encode("utf-8")
                else:
                    payload = f"{value:.{self.precision}f}\n".encode("utf-8")

            elif self.udp_format == "json":
                obj = {"state": value}
                if self.include_ts:
                    obj["t"] = t
                payload = (json.dumps(obj) + "\n").encode("utf-8")

            else:  # "binary"
                # Network byte order (big endian). Either float only, or double timestamp + float.
                if self.include_ts:
                    payload = struct.pack("!df", t, value)  # double (8B) + float (4B)
                else:
                    payload = struct.pack("!f", value)      # float (4B)

            self.sock.sendto(payload, self.dest)

        except Exception as e:
            # Non-blocking socket can raise on network hiccups; keep node alive
            self.get_logger().warn(f"UDP send failed: {e}")

    # ---- Dynamic params ----

    def _on_param_change(self, params):
        changed = False
        for p in params:
            if p.name == "dest_host" and p.value != self.dest_host:
                self.dest_host = str(p.value); changed = True
            elif p.name == "dest_port" and int(p.value) != self.dest_port:
                self.dest_port = int(p.value); changed = True
            elif p.name == "udp_format":
                val = str(p.value).lower()
                if val in ("text", "json", "binary"):
                    self.udp_format = val
                    self.get_logger().info(f"udp_format -> {self.udp_format}")
                else:
                    self.get_logger().warn(f"Ignored invalid udp_format '{p.value}'")
            elif p.name == "include_timestamp":
                self.include_ts = bool(p.value)
                self.get_logger().info(f"include_timestamp -> {self.include_ts}")
            elif p.name == "precision":
                self.precision = int(p.value)
                self.get_logger().info(f"precision -> {self.precision}")
            elif p.name == "broadcast":
                new_broadcast = bool(p.value)
                if new_broadcast != self.broadcast:
                    self.broadcast = new_broadcast
                    try:
                        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1 if self.broadcast else 0)
                        self.get_logger().info(f"broadcast -> {self.broadcast}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to set SO_BROADCAST: {e}")

        if changed:
            self.dest = (self.dest_host, self.dest_port)
            self.get_logger().info(f"Dest -> {self.dest_host}:{self.dest_port}")

        # rclpy requires returning a SetParametersResult-like object; using simple OK
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    # ---- Cleanup ----

    def destroy_node(self):
        try:
            if hasattr(self, "sock") and self.sock:
                self.sock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrainUDPStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
