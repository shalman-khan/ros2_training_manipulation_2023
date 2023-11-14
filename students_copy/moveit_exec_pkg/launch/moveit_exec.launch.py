from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    
    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="moveit_exec_interface",
        package="moveit_exec_pkg",
        executable="moveit_exec_interface",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_demo])