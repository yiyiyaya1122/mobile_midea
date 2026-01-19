#!/usr/bin/env python3
"""
Launch file to start VR controller node and torso VR control node simultaneously
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # VR Controller Node - publishes VR pose and button data
        Node(
            package='quest3s_vr',
            executable='controller_node',
            name='quest3s_node',
            output='screen',
            parameters=[],
        ),
        
        # Torso VR Control Node - controls robot torso via VR joystick
        Node(
            package='quest3s_vr',
            executable='torso_chassis_vr_control',
            name='torso_chassis_vr_control',
            output='screen',
            parameters=[],
        ),
    ])

