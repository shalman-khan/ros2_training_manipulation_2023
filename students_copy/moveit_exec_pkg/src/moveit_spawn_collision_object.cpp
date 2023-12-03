#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include<chrono>
#include <ros2_aruco_interfaces/srv/spawn_collision_object.hpp>


using namespace std::chrono_literals;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("movet_exec_interface");


  ///////////////////////////////////////////////////////////////////
  // Excercise 4.1: Write a client request funcion to get detection Aruco Position
  // Use the below struct retrieve the result 
  ///////////////////////////////////////////////////////////////////


struct CollisionServiceResult {
  geometry_msgs::msg::Pose obj_pose;
  bool detect_id;
};

CollisionServiceResult detect_collision_object(
  rclcpp::Node::SharedPtr demo_node)
{
  // CODE HERE:
  // Service Name: spawn_collision_object

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

}


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

  rclcpp::Node::SharedPtr collision_node =
  rclcpp::Node::make_shared("collision_client", "", node_options);

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Define Joint Model Group to extract the set of joints to control from specific planning group
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Class to describe the Planning Scene [Helps to add objects to the planning environment]
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Current set of Joint Values
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);



  geometry_msgs::msg::Pose target_pose1;

  target_pose1.orientation.w = 0.0;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;

  target_pose1.position.x = 0.6;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.97;


  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  auto success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.execute(plan);
  move_group.move();



  ///////////////////////////////////////////////////////////////////
  // Excercise 4.2: Call the detect_collision_object to get Marker pose. 
  // Use the result when detect_id is TRUE
  ///////////////////////////////////////////////////////////////////
    //  CODE HERE
    // CollisionServiceResult aruco_collision_detect;
    // aruco_collision_detect= detect_collision_object(collision_node);


  ///////////////////////////////////////////////////////////////////
  // Excercise 4.3: Create a box for following dimensions and use the detected pose as box pose
  // Set frame id as "camera_pose_link"
  // Add the detected collision object to the planning scene   
  ///////////////////////////////////////////////////////////////////

  //Set the box dimensions to 0.065m(x), 0.119m(y), 0.04m(z)



  RCLCPP_INFO(LOGGER, "Add an object into the world");

  rclcpp::shutdown();
  return 0;
}
