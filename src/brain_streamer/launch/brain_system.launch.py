"""
Launch file to start both the brain_streamer and brain_processor nodes.

This launch file starts:
- brain_stream: Reads EEG data from Muse headset via LSL and publishes to /brain/raw
- brain_processor: Processes raw EEG data and publishes mental state actions

Usage:
    ros2 launch brain_streamer brain_system.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with brain_streamer and brain_processor nodes."""
    
    # Brain Streamer Node - reads from Muse headset
    brain_stream_node = Node(
        package='brain_streamer',
        executable='brain_stream',
        name='brain_stream',
        output='screen',
        emulate_tty=True,
    )
    
    # Brain Processor Node - processes EEG data
    brain_processor_node = Node(
        package='brain_processor',
        executable='brain_processor',
        name='brain_processor',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        brain_stream_node,
        brain_processor_node,
    ])
