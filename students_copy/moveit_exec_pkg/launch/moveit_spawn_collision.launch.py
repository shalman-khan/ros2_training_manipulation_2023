from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    
    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="moveit_spawn_collision",
        package="moveit_exec_pkg",
        executable="moveit_spawn_collision_object",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    ld = LaunchDescription([move_group_demo])

    # Tf transformer to generate a Eye in Hand Camera Frame
    ld.add_action(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0.1', '-0.1', '0.0', 
                       '0.0', '0.0', '0.0', 
                       '1.0', 'panda_link8', 'camera_pose_link']
    ))

    # ROS2 aruco node to aruco detection
    ld.add_action(Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen'
    ))

    return ld