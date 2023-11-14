## ROS-I Manipulation Training Day 1 Exercise 3: Use Move Group and Move Group Interface to execute set of motion plan tasks

There are three objectives to this exercise

1) Pose Goal and Plan
2) Add Collision Object and Plan
3) Joint Goal and Plan


Let's first open the executable to be edited for this exercise

![Folder_img](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_exe_folder_structure.png)



```
cd ~/day1_ws/src
code moveit_exec_pkg/src/moveit_exec_interface.cpp
```

For this exercise, 
1) Editing will be done in ***moveit_exec_interface.cpp***
2) And Launch ***moveit_exec.launch.py***

<br>

### Objective 1: Set Pose Goal and plan

  1) Identify the planning group name defined in the panda_moveit_pkg 
  2) Use the "geometry_msgs::msg::Pose" to define the Target pose

  Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html

Please refer to the template below to understand where editings are needed

![Pose Goal Template](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/pose_goal_template.png)

### Terminal 1: [Launch Move Group: Move Group will be ready to accept motion plan requests and obstacle additions]
```
cd ~/day1_ws
colcon build --packages-select moveit_exec_pkg
source install/setup.bash
ros2 launch panda_moveit_config move_group.launch.py 
```

### Terminal 2: [Launch Robot State Publisher: Publishes current robot state]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config rsp.launch.py
```

### Terminal 3: [Launch Rviz: Publishes current robot state]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config moveit_rviz.launch.py
```

Now you should see the below window opened which loads motion planning toolbox, robot model based on robot state publisher information

![Rviz](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/rviz_moveit.png)


### Terminal 4: [Launch Moveit Interface: Calls Moveit group with motion plan and collision requests]

```
source ~/day1_ws/install/setup.bash
ros2 launch moveit_exec_pkg moveit_exec.launch.py
```

Now you should notice the robot planning (Note: not moving) from current pose to target pose

### Objective 2: Add Collision Object and plan

  1) Copy and paste the collision section from the Readme document
  2) Use the "geometry_msgs::msg::Pose" to define the Collision Box pose
 
  Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  
  Define position-> [x: 0.48, y:0.0; z:0.72] orientation->[w:1.0]

Please refer to the template below to understand where editings are needed

![Add Collision Template](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/add_collision_template.png)


Please copy below contents to **"Code: add collision objects section here"** 

```
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "collision_box";

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // 2) Use the "geometry_msgs::msg::Pose" to define the Collision Box pose
  // Refer: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
  // Define position-> [x: 0.48, y:0.0; z:0.72] orientation->[w:1.0]  geometry_msgs::msg::Pose box_pose;

  geometry_msgs::msg::Pose box_pose;
  // Code: add the Collision Box pose here  
  // ......................
  // ......................
  // ......................



  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  RCLCPP_INFO(LOGGER, "Collision Object added to Planning Scene");
  planning_scene_interface.addCollisionObjects(collision_objects);

```

### Terminal 1: [Launch Move Group: Move Group will be ready to accept motion plan requests and obstacle additions]
```
cd ~/day1_ws
colcon build --packages-select moveit_exec_pkg
source install/setup.bash
ros2 launch panda_moveit_config move_group.launch.py 
```

### Terminal 2: [Launch Robot State Publisher: Publishes current robot state]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config rsp.launch.py
```

### Terminal 3: [Launch Rviz: Publishes current robot state]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config moveit_rviz.launch.py
```

Now you should see the below window opened which loads motion planning toolbox, robot model based on robot state publisher information


### Terminal 4: [Launch Moveit Interface: Calls Moveit group with motion plan and collision requests]

```
source ~/day1_ws/install/setup.bash
ros2 launch moveit_exec_pkg moveit_exec.launch.py
```

Now you should notice the robot planning (Note: not moving) from current pose to target pose without collision with the added collision object

### Objective 3: Set Joint Goal and Plan

  1) Copy and paste the Joint Goal section from the Readme document


Please refer to the template below to understand where editings are needed

![Add Joint Goal Template](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/joint_goal_template.png)


Please copy below contents to **"Code: Add Joint Goal section here"** 

```
  // Extract current joint group position and copy it to a variable
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  joint_group_positions[0] = -1.0;  // radians
  joint_group_positions[1] = -0.32;
  joint_group_positions[2] = 0.016;
  joint_group_positions[3] = -0.4945;
  joint_group_positions[4] = -0.1441;
  joint_group_positions[5] = 0.4824;
  joint_group_positions[6] = 0.7844;
  move_group.setJointValueTarget(joint_group_positions);


  // Ackowledgement to verify the plan's success
  ack = (move_group.plan(moveit_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing JOINT GOAL plan %s", ack ? "" : "Unsuccessful");
```

### Terminal 1: [Launch Move Group: Move Group will be ready to accept motion plan requests and obstacle additions]
```
cd ~/day1_ws
colcon build --packages-select moveit_exec_pkg
source install/setup.bash
ros2 launch panda_moveit_config move_group.launch.py 
```

### Terminal 2: [Launch Robot State Publisher: Publishes current robot state]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config rsp.launch.py
```

### Terminal 3: [Launch Rviz: Publishes current robot state]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config moveit_rviz.launch.py
```

Now you should see the below window opened which loads motion planning toolbox, robot model based on robot state publisher information

![Rviz](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/rviz_moveit.png)


### Terminal 4: [Launch Moveit Interface: Calls Moveit group with motion plan and collision requests]

```
source ~/day1_ws/install/setup.bash
ros2 launch moveit_exec_pkg moveit_exec.launch.py
```

Now you should notice the robot planning (Note: not moving) from current joint position to target joint goal. 
