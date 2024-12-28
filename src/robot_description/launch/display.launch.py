from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_robot', '-file', 'install/robot_description/share/robot_description/urdf/robot.urdf', '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'install/robot_description/share/robot_description/config/rviz_config.rviz']
        )
    ])
