import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. Tentukan Path ke Paket
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_control = get_package_share_directory('robot_control')

    # 2. Proses File XACRO (URDF baru Anda)
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # 3. Node untuk Robot State Publisher (Mengirimkan TF robot berdasarkan URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True}]
    )

    # 4. Jalankan Gazebo
    # Kita perlu menjalankan server Gazebo (gzserver) dan klien (gzclient)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 5. Spawn Robot ke Gazebo
    # Ini adalah node yang memanggil layanan /spawn_entity di Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'mobile_robot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1'],
        output='screen'
    )

    # 6. Spawn Target (Plate) ke Gazebo
    plate_sdf_path = os.path.join(pkg_robot_description, 'models', 'plate.sdf')
    spawn_plate_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', plate_sdf_path,
                   '-entity', 'green_plate',
                   '-x', '2.0',  # Spawn 2 meter di depan robot
                   '-y', '0.0',
                   '-z', '0.01'],
        output='screen'
    )

    # 7. Jalankan File Launch Kontrol (object_detection, pid, movement)
    control_launch_file = os.path.join(pkg_robot_control, 'launch', 'control.launch.py')
    control_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file)
    )

    # Rantai Aksi: Kita ingin spawn robot HANYA SETELAH Gazebo berjalan.
    # Kita akan spawn plate dan menyalakan kontrol setelah robot berhasil di-spawn.
    # (Catatan: Untuk kesederhanaan, kita spawn semuanya bersamaan setelah Gazebo. 
    #  Dalam sistem yang lebih kompleks, kita menggunakan Event Handlers)

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot_node,
        spawn_plate_node,
        control_nodes
    ])
