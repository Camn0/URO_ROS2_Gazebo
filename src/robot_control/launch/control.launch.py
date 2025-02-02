#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='object_detection',
            name='object_detection',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='movement_control',
            name='movement_control',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
