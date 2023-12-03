#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("movet_exec_interface");

int main(int argc, char** argv)
{

  //  Initializes the ROS C++ client library
  rclcpp::init(argc, argv);

  // Options to customize the behavior of the node
  rclcpp::NodeOptions node_options;

  // Sets an option for the node to automatically declare parameters from overrides [Node name: movet_exec_interface]
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("movet_exec_interface", node_options);

  // Spins up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;

  // Adds the node to executor
  executor.add_node(move_group_node);

  // Detach makes the thread to run individually in the background 
  std::thread([&executor]() { executor.spin(); }).detach();
  
  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 1: Planning to a POSE GOAL 
  // 1) Identify the planning group name defined in the panda_moveit_pkg 
  // 2) Use the "geometry_msgs::msg::Pose" to define the Target pose
  // Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  ////////////////////////////////////////////////////////////////////

  // Planning group Definition: To plan and control specfic set of joints defined in the planning group
  // 1) Identify the planning group name defined in the panda_moveit_pkg 

  // CODE HERE

  // Define Joint Model Group to extract the set of joints to control from specific planning group
  // CODE HERE

  // Class to describe the Planning Scene [Helps to add objects to the planning environment]
  // CODE HERE


  // Current set of Joint Values
  // CODE HERE

  // 2) Use the "geometry_msgs::msg::Pose" to define the Target pose
  // Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  // Define position-> [x: 0.7, y:0.0; z:0.97] orientation->[w:0.0, x:-1.0, y:0.0, z:0.0]
  //  Target Pose Defintion 
  // CODE HERE
  // Code: add the Pose Goal here  
  // ......................
  // ......................
  // ......................

  // Set the Target Pose
  // CODE HERE


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
  // CODE HERE

  // Ackowledgement to verify the plan's success

  // CODE HERE


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
