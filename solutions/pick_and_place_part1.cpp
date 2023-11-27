/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

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

#include "robotiq_msgs/srv/c_model_response.hpp"
#include <ros2_aruco_interfaces/srv/spawn_collision_object.hpp>


using namespace std::chrono_literals;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

  ///////////////////////////////////////////////////////////////////
  // Write a client request funcion to activate Gripper based on defined width
  //  [refer to day3 exercise]
  ///////////////////////////////////////////////////////////////////

void activate_gripper(
  rclcpp::Node::SharedPtr demo_node,
  int gripper_width){
    rclcpp::Client<robotiq_msgs::srv::CModelResponse>::SharedPtr client =
      demo_node->create_client<robotiq_msgs::srv::CModelResponse>("/gripper_service");

    auto request = std::make_shared<robotiq_msgs::srv::CModelResponse::Request>();
    request->configuration = 1;
    request->robotiq2f_type = 85;
    request->rpr = gripper_width;
    request->rsp = 255;
    request->rfr = 255;
    request->rmod = 0;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(demo_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto service_response = result.get();

      RCLCPP_INFO(LOGGER, "Service Sucess");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to call service");
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

  ///////////////////////////////////////////////////////////////////
  // Write a client request funcion to get object pose based on defined marker id [use below struct to extact values]
  // Service name: SpawnCollisionObject [/spawn_collision_object]
  // Service type: ros2_aruco_interfaces/srv/SpawnCollisionObject
  // Command line: ros2 service call /spawn_collision_object ros2_aruco_interfaces/srv/SpawnCollisionObject '{marker_id: 45}'
  ///////////////////////////////////////////////////////////////////

struct CollisionServiceResult {
  geometry_msgs::msg::Pose obj_pose;
  bool detect_id;
};

CollisionServiceResult detect_collision_object(
  rclcpp::Node::SharedPtr demo_node,
  int id_to_detect)
{
  // CODE HERE:
  rclcpp::Client<ros2_aruco_interfaces::srv::SpawnCollisionObject>::SharedPtr client =
    demo_node->create_client<ros2_aruco_interfaces::srv::SpawnCollisionObject>("spawn_collision_object");

  auto request = std::make_shared<ros2_aruco_interfaces::srv::SpawnCollisionObject::Request>();
  request->marker_id = id_to_detect;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return {};  // Returning an empty result in case of an error.
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(demo_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto service_response = result.get();
    RCLCPP_INFO(LOGGER, "Service Success");

    // Assuming `service_response` has information about collision detection.
    CollisionServiceResult detection_result;
    detection_result.obj_pose = service_response->obj_pose;
    detection_result.detect_id = service_response->detect_id;

    return detection_result;
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to call service");
    return {};  // Returning an empty result in case of a failure.
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

 
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::Node::SharedPtr gripper_node =
    rclcpp::Node::make_shared("gripper_client", "", node_options);

  rclcpp::Node::SharedPtr spawn_node =
    rclcpp::Node::make_shared("gripper_client", "", node_options);


  // Define Planning Groups

  static const std::string PLANNING_GROUP = move_group_node->get_parameter("planning_group").as_string();
  static const std::string HAND_PLANNING_GROUP = move_group_node->get_parameter("end_effector_planning_group").as_string();
  static const std::string BASE_LINK = move_group_node->get_parameter("base_link").as_string();
  std::vector< std::string> gripper_fingers = move_group_node->get_parameter("end_effector_finger_links").as_string_array();

 // Define Move group with defined Planning Group
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Move Group Planner Selection 
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("LBKPIECEkConfigDefault");  


  // Define Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Define Joint Model Group with defined Planning Group
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

   //  Define the plan using planning interface
  moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;

  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 1: Move Robot to defined "tray1_pose" as starting position [open gripper at initial step]
  // 1) Scan Aruco Tag to generate Tray 1 as collision object

  ////////////////////////////////////////////////////////////////////

  activate_gripper(
    gripper_node,
    85);

  move_group.setNamedTarget("tray1_pose");

  bool tray1_ack = (move_group.plan(moveit_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  move_group.execute(moveit_plan);
  move_group.move();

  CollisionServiceResult tray1_collision_detect;

  int tray1_id = 35; 

  bool detect_tray1  = false;
  while (!detect_tray1) {
    tray1_collision_detect= detect_collision_object(spawn_node, tray1_id);
    detect_tray1 = tray1_collision_detect.detect_id;
  }


  moveit_msgs::msg::CollisionObject tray1_collision_object;
  tray1_collision_object.header.frame_id = "camera_color_optical_frame";
  tray1_collision_object.id = "tray1";

  shapes::Mesh * tray1_import_mesh = shapes::createMeshFromResource("package://robotiq_85_description/meshes/visual/tray.dae");

  shape_msgs::msg::Mesh tray1_mesh;
  shapes::ShapeMsg tray1_mesh_msg;
  shapes::constructMsgFromShape(tray1_import_mesh,tray1_mesh_msg);
  tray1_mesh = boost::get<shape_msgs::msg::Mesh>(tray1_mesh_msg);

  tray1_collision_object.meshes.push_back(tray1_mesh);

    RCLCPP_INFO(LOGGER, "Add TRAY1 object into the world");  

  tray1_collision_object.mesh_poses.push_back(tray1_collision_detect.obj_pose);
  tray1_collision_object.operation = tray1_collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> tray1_collision_objects;
  tray1_collision_objects.push_back(tray1_collision_object);

  planning_scene_interface.addCollisionObjects(tray1_collision_objects);


  rclcpp::shutdown();
  return 0;
}
