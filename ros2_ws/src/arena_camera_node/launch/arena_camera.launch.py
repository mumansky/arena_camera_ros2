"""
Launch file for Arena Camera ROS2 Node with YAML configuration.

This launch file demonstrates how to start the arena_camera_node with
parameters loaded from a YAML configuration file.

Usage:
    ros2 launch arena_camera_node arena_camera.launch.py

    # Or with a custom config file:
    ros2 launch arena_camera_node arena_camera.launch.py config:=/path/to/custom_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
            FindPackageShare('arena_camera_node'),
            'config',
            'arena_camera.yaml'
        ]),
        description='Path to the YAML configuration file'
    )

    # Arena Camera Node
    arena_camera_node = Node(
        package='arena_camera_node',
        executable='start',
        name='arena_camera_node',
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        arena_camera_node,
    ])
