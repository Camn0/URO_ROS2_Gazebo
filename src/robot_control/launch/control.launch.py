from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='object_detection.py',
            output='screen',
        ),
        Node(
            package='robot_control',
            executable='movement_control.py',
            output='screen',
        ),
    ])
