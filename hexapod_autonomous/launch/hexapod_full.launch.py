#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware',
            default_value='true',
            description='Enable hardware mode'
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_bringup',
            name='hexapod_bringup',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_control',
            name='servo_controller',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_gait',
            name='gait_controller',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_vision',
            name='camera_node',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_object_detect',
            name='object_detector',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='face_recognition',
            name='face_recognition_node',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_ultrasonic',
            name='ultrasonic_node',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_navigation',
            name='navigator',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='person_follower',
            name='person_follower',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_voice',
            name='voice_node',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='hexapod_ai_brain',
            name='ai_brain',
            output='screen',
        ),
        
        Node(
            package='hexapod_autonomous',
            executable='memory_node',
            name='memory_node',
            output='screen',
        ),
    ])
