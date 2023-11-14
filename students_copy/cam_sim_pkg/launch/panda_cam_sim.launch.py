import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    
    cam_sim_pkg = os.path.join(
        get_package_share_directory('cam_sim_pkg'),
        'worlds',
        'panda_arm_world.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': cam_sim_pkg}.items(),
             )

    panda_description_path = os.path.join(
        get_package_share_directory('cam_sim_pkg'))

    pkg_name = 'cam_sim_pkg'
    file_subpath = 'urdf/panda.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] 
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'panda'],
                        output='screen')

    return LaunchDescription([
        gazebo, 
        node_robot_state_publisher,
        spawn_entity,
    ])