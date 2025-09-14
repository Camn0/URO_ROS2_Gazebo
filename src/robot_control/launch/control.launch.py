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
            executable='pid_x',
            name='pid_x',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='pid_y',
            name='pid_y',
            output='screen'
        ),
         Node(
            package='robot_control',
            executable='movement_control',
            name='movement_control',
            output='screen',
            # BLOK BARU INI MENINGKATKAN KECEPATAN PENCARIAN/PEMINDAIAN:
            parameters=[
                {"search_angular_velocity": 0.8} # Defaultnya 0.3, kita ubah jadi 0.8
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
