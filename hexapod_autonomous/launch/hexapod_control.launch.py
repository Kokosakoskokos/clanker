#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_autonomous',
            executable='hexapod_control',
            name='servo_controller',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='hexapod_gait',
            name='gait_controller',
            output='screen',
            parameters=[],
        ),
    ])
