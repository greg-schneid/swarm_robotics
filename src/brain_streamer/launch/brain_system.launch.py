"""
Launch file to start both the brain_streamer and brain_processor nodes.

This launch file starts:
- brain_stream: Reads EEG data from Muse headset via LSL and publishes to /brain/raw
- brain_processor: Processes raw EEG data and publishes mental state actions

Usage:
    ros2 launch brain_streamer brain_system.launch.py
    ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true
    ros2 launch brain_streamer brain_system.launch.py save_to_csv:=true csv_output_dir:=/path/to/output
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with brain_streamer and brain_processor nodes."""
    
    # Declare launch arguments
    save_to_csv_arg = DeclareLaunchArgument(
        'save_to_csv',
        default_value='false',
        description='Enable CSV logging of raw brain data'
    )
    
    csv_output_dir_arg = DeclareLaunchArgument(
        'csv_output_dir',
        default_value='/workspaces/swarm_robotics/data',
        description='Directory path for CSV output files'
    )
    
    # Brain Streamer Node - reads from Muse headset
    brain_stream_node = Node(
        package='brain_streamer',
        executable='brain_stream',
        name='brain_stream',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'save_to_csv': LaunchConfiguration('save_to_csv'),
            'csv_output_dir': LaunchConfiguration('csv_output_dir'),
        }]
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
        save_to_csv_arg,
        csv_output_dir_arg,
        brain_stream_node,
        brain_processor_node,
    ])
