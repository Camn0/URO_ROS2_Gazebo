#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='a_star',
            name='a_star_navigation',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
