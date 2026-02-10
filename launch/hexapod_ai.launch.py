#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_autonomous',
            executable='hexapod_voice',
            name='voice_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='hexapod_ai_brain',
            name='ai_brain',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='memory_node',
            name='memory_node',
            output='screen',
            parameters=[],
        ),
    ])
