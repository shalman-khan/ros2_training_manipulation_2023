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
  // CODE HERE:
   ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////

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
   ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
}

int main(int argc, char** argv)
{
  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 0: Refer to Day 3 Exercise 6 pick_and_place.cpp for initialization

  ////////////////////////////////////////////////////////////////////

  // Initialize the ROS 2 client library

  // Create node options

  // Automatically declare parameters from overrides

  // Create a ROS 2 node for Move group interface

  // Create an executor to handle node execution

  // Add the Move group interface node to the executor

  // Start a separate thread to spin the executor

  // Create a separate ROS 2 node for the gripper client



  // Define Planning Groups

  // Define Move group with defined Planning Group

  // Move Group Planner Selection 



  // Define Planning scene interface

  // Define Joint Model Group with defined Planning Group

  // Define the plan using planning interface

  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 1: Move Robot to defined "tray1_pose" as starting position [open gripper at initial step]
  // 1) Scan Aruco Tag to generate Tray 1 as collision object

  ////////////////////////////////////////////////////////////////////

  // CODE HERE:
   ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////


  // <ACTIVATE GRIPPER OPEN>


  // <SET NAMED TARGET AND MOVE TO "tray1_pose". Refer to below link
  // http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c

  // <SPAWN COLLISION OBJECT BY SENDING CLIENT REQUEST USING CREATED FUNCTION> 
  // Determine the tray1 pose with respect to the detected aruco marker
  // mesh file located in robotiq_85_description/meshes/visual/tray.dae
  // refer to template file "collision_mesh_import_template.txt" to import mesh


  // MARKER ID for tray1: 35

  // <ADD COLLISION OBJECT TO PLANNING INTERFACE>


  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 2: Move Robot to defined "tray2_pose"  
  // 1) Scan Aruco Tag to generate Tray 2 as collision object

  ////////////////////////////////////////////////////////////////////

  // CODE HERE:
   ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////

  // <SET NAMED TARGET AND MOVE TO "tray2_pose". Refer to below link
  // http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c

  // <SPAWN COLLISION OBJECT BY SENDING CLIENT REQUEST USING CREATED FUNCTION> 
  // Determine the tray2 pose with respect to the detected aruco marker
  // mesh file located in robotiq_85_description/meshes/visual/tray.dae
  // refer to template file "collision_mesh_import_template.txt" to import mesh

  // MARKER ID for tray2: 25

  // <ADD COLLISION OBJECT TO PLANNING INTERFACE>

  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 3.1: Detect the Aruco Pick Box  
  // 1) Move Robot to defined "tray1_pose"  
  // 2) Scan Aruco Tag to generate Aruco Box as collision object

  ////////////////////////////////////////////////////////////////////


  // <SET NAMED TARGET AND MOVE TO "tray1_pose". Refer to below link

  // <SPAWN COLLISION OBJECT BY SENDING CLIENT REQUEST USING CREATED FUNCTION> 
  // MARKER ID for aruco pick box: 45


  // define a box similar to the one in Day3 Excercise 6 pick_and_place.cpp
  //Set the box dimensions to 0.065m(x), 0.119m(y), 0.04m(z)



  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 3.2: Pick the detected Aruco Box
  // 1) Make sure Gripper is open before executing the plan
  // 2) Use detected pose to generate cartesian path plan to pick the box [refer to day3 exercise 6 pick_and_place.cpp for cartesian plan]
  // 3) Move Robot to defined "tray1_pose" with picked object

  ////////////////////////////////////////////////////////////////////
 // CODE HERE:


  ///////////////////////////////////////////////////////////////////
  // OBJECTIVE 3.3: Place the picked Aruco Box in tray2
  // 1) Make sure Gripper is open before executing the plan
  // 2) Use detected pose to generate cartesian path plan to pick the box [refer to day3 exercise 6 pick_and_place.cpp for cartesian plan]
  // 3) Move Robot to defined "tray1_pose" with picked object

  ////////////////////////////////////////////////////////////////////

 // CODE HERE:

  rclcpp::shutdown();
  return 0;
}