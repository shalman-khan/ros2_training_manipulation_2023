#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("movet_exec_interface");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("movet_exec_interface", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 1: Planning to a POSE GOAL 
  // 1) Identify the planning group name defined in the panda_moveit_pkg 
  // 2) Use the "geometry_msgs::msg::Pose" to define the Target pose
  // Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  ////////////////////////////////////////////////////////////////////

  // Planning group Definition: To plan and control specfic set of joints defined in the planning group
  // 1) Identify the planning group name defined in the panda_moveit_pkg 
  static const std::string PLANNING_GROUP = "identify planning group";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Define Joint Model Group to extract the set of joints to control from specific planning group
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Class to describe the Planning Scene [Helps to add objects to the planning environment]
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Current set of Joint Values
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  // 2) Use the "geometry_msgs::msg::Pose" to define the Target pose
  // Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  // Define position-> [x: 0.7, y:0.0; z:0.97] orientation->[w:0.0, x:-1.0, y:0.0, z:0.0]
  //  Target Pose Defintion 
  geometry_msgs::msg::Pose pose_goal;
  // Code: add the Pose Goal here  
  // ......................
  // ......................
  // ......................

  // Set the Target Pose
  move_group.setPoseTarget(pose_goal);


  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 2: Add collision box to planning scene 
  // 1) Copy and paste the collision section from the Readme document
  // 2) Use the "geometry_msgs::msg::Pose" to define the Collision Box pose
  // Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  // Define position-> [x: 0.48, y:0.0; z:0.72] orientation->[w:1.0]
  ////////////////////////////////////////////////////////////////////
 
  // Code: add Collision object section here  
  // ......................
  // ......................
  // ......................



  //////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////

  //  Planning the plan and Visualization [Not MOVE]
  moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;

  // Ackowledgement to verify the plan's success
  bool ack = (move_group.plan(moveit_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing POSE GOAL plan %s", ack ? "" : "Unsuccessful");

  // Before this step: Planning is Completed
  // The following step executes the plan
  // To Execute the plan a controller should be launched
  // move_group.move();



  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 3: Plan to a Joint GOal 
  // 1) Copy and paste the Joint Goal section from the Readme document
  ////////////////////////////////////////////////////////////////////

  // Extract current joint group position and copy it to a variable

  // Code: add Joint Goal section here  
  // ......................
  // ......................
  // ......................

  // Before this step: Planning is Completed
  // The following step executes the plan
  // To Execute the plan a controller should be launched
  // move_group.move();


  rclcpp::shutdown();
  return 0;
}
