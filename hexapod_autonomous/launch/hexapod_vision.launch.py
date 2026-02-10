#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_autonomous',
            executable='hexapod_vision',
            name='camera_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='hexapod_object_detect',
            name='object_detector',
            output='screen',
            parameters=[],
        ),
        Node(
            package='hexapod_autonomous',
            executable='face_recognition',
            name='face_recognition_node',
            output='screen',
            parameters=[],
        ),
    ])
