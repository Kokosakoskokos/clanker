#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_autonomous',
            executable='hexapod_ultrasonic',
            name='ultrasonic_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='hexapod_navigation',
            name='navigator',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='person_follower',
            name='person_follower',
            output='screen',
            parameters=[],
        ),
    ])
