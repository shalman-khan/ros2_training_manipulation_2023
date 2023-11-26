from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import  LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition

def generate_launch_description():
    launch_description = LaunchDescription()

    sim_only_value = LaunchConfiguration('sim_only')
 
    launch_description.add_action(
        DeclareLaunchArgument(
            'sim_only',
            default_value="true"
        )
    )
    gripper_config = os.path.join( 
        get_package_share_directory('robotiq_ros_service'), 'config', 'config.yaml')
    # gripper service server
    finger_gripper_server_node = Node(
        package='robotiq_ros_service',
        executable='robotiq_ros2_server',
        name='robotiq_ros2_server',
        output='screen',
        parameters=[
            gripper_config,
            {"sim_only" : sim_only_value}]
    )
    # real_gripper_hardware = Node(
    #     package="robotiq_urcap_control",
    #     executable="cmodel_urcap_driver",
    #     output="screen",
    #     emulate_tty=True,
    #     condition=UnlessCondition(sim_only_value),
    #     parameters=[
    #         {'gripper_service': "robotiq_urcap_control"},
    #         {'ip_address': "192.168.0.30"}]
    # )

    launch_description.add_action(finger_gripper_server_node)
    return launch_description
