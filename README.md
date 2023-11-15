## ROS-I Manipulation Training Day 1 Exercise 1: Visualize Robot arm and Add Table and Camera

There are two objectives to this exercise

1) Add the right joint for the gripper and visualize the robot arm in RViz
2) Add a table, camera and visualize in RViz
   
![Exercise1](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/Exercise1_image1.png)

<br>

### Before Beginning, Create a Workspace 
```
mkdir -p ~/day1_ws/src
cp -r <day1_contents> ~/day1_ws/src
```

## Objective 1: Add the right joint for the gripper and visualize the robot arm 
use gedit or code to open code editors

```
cd ~/day1_ws/src
code arm_urdf_pkg/urdf/panda.urdf
```

Now you should notice a urdf file similar to the one shown below

![Urdf_img](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/urdf_file_image.png)

Move to the **<panda_finger_joint1>** and **<panda_finger_joint2>** tags

![Identify_joint](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/joint_type.png)

Identify the joint type for the **Finger Gripper Joints**.

Once done, now it is time to visualize your robot description file in RViz. 
Before visualizing, you should build and source to visualize the changes. 

```
cd ~/day1_ws
colcon build --packages-select arm_urdf_pkg
source install/setup.bash
ros2 launch arm_urdf_pkg robot_arm_visual.launch.py
```

Now you should be able to see the robot similar to the image below

![Identify_joint](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/first_viz.png)

Change fixed frame to **"panda_link0"**

Use Sliders in Joint State Publisher box to visualize robot in different joint states [or press randomize]

## Objective 2: Add a table, camera and visualize

Scroll down to the same **panda.urdf** file 

### Add Table Link and Joint

1) Step 1: Create a link with name "table"
2) Step 2: Define Visual Component with origin **[rpy="1.57 0 0" xyz="0 0 0"]**
3) Step 3: Define Geoemtry with mesh file "workbench.dae" located in meshes/visual folder of the package.
4) Step 4: Repeat Step 1 to Step 3 for Collision Component.
5) Step 5: Create a joint with name "table_link_joint" and type "fixed"
6) Step 6: Define origin **rpy="0 0 0"** and determine **xyz** for the joint [hint: Table height from its origin -> 0.468m]
7) Step 7: Define Parent link as "table" and Child link as "panda_link0". 

### Add Camera Link and Joint

1) Step 1: Create a link with name "camera_link"
2) Step 2: Define Visual Component with origin **[rpy="1.57 0 1.57" xyz="0.0 0 0.0"]**
3) Step 3: Define Geoemtry with mesh file "workbench.dae" located in meshes/visual folder of the package.
4) Step 4: Repeat Step 1 to Step 3 for Collision Component.
5) Step 5: Create a joint with name "base_to_static_camera" and type "fixed"
6) Step 6: Define joint origin **[rpy="0 0 0" xyz="1.0 0 1.0"]**
7) Step 7: Define Parent link as "panda_link0" and Child link as "camera_link".


Once done, now it is time to visualize your robot description file in RViz. 
Before visualizing, you should build and source to visualize the changes. 

```
cd ~/day1_ws
colcon build --packages-select arm_urdf_pkg
source install/setup.bash
ros2 launch arm_urdf_pkg robot_arm_visual.launch.py
```

Note: Change fixed frame to **"table"**


Now you should be able to see the robot scene similar to the image below

![Exercise1](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/Exercise1_image1.png)


<br>


## ROS-I Manipulation Training Day 1 Exercise 2: Use Moveit Setup Assistant to Develop Moveit Config Package


For this exercise, 
1) **Moveit Setup Assistant** will be used to create Moveit Config

```
cd ~/day1_ws
ros2 launch moveit_setup_assistant setup_assistant.launch.py 
```

#### Step 1: Click ***"Create New Moveit Configuration Package"*** and select and load the created URDF file "panda.urdf" from first excercise. 

![Step 1](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup1.png)

Now you should see a robot model loaded at the right display window

![Step 2](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup2.png)


#### Step 2: Click Generate Collision Matric

![Step 3](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup3.png)

Now you should see a list of link pairs with enable/disable collision and reasons to disable collision

![Step 4](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup4.png)



#### Step 3: Define Robot Arm Planning Group
* Click "**add group**" 
* Goup Name: "**panda_arm**"
* Kinematic Solver: "**KDL Kinematics Plugin**"
* Group Default Planner: None 
* Click "**Add Joints**"

![Step 5](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup5.png)

* Now you should see list of joint names and select the highlighted group of joints and press "**>**" button.
* Once done, click "**save**".

![Step 6](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup6.png)

* Now you should see the added planning group with selected joints in main planning group window

![Step 7](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup7.png)

#### Step 4: Define Robot Arm Planning Group
* Click "**add group**" 
* Goup Name: "**hand**"
* Kinematic Solver: "**None**"
* Group Default Planner: None 
* Click "**Add Links**"

![Step 8](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup8.png)

* Now you should see list of link names and select the highlighted group of links and press "**>**" button.
* Once done, click "**save**".

![Step 9](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup9.png)

* Now you should see the added planning group with selected links in main planning group window

![Step 10](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup10.png)

#### Step 5: Define Robot Home Pose

* Name pose as "**home**"
* Select "**panda_arm**" group 
* Select a random valid pose using sliders to set home position.


![Step 11](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup11.png)


#### Step 6: Define Gripper Pose

* Name pose as "**open**"
* Select "**hand**" group 
* Select a random valid pose using sliders to set gripper open position [prefer: 0.0100]

![Step 12](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup12.png)

* Name pose as "**close**"**
* Select "**hand**" group 
* Select a random valid pose using sliders to set gripper closed position [prefer: 0.0]

![Step 13](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup13.png)

#### Step 7: Set End Effectors

* Name End Effector as "**hand**"
* Select "**hand**" as end effector group 
* Select "**panda_link8**" as parent link
* leave parent group blank

![Step 14](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup14.png)

#### Step 8: Skip Passive Joints selection

![Step 15](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup15.png)

#### Step 9: ROS2 Control URDF Modification

* Select position as command interface
* Select position and velocity as state interface

![Step 16](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup16.png)


#### Step 10: Set up ROS2 Controllers for Robot Arm

* Click "**add controller**" 

![Step 17](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup17.png)

* Controller Name: "**panda_arm_controller**"
* Controller Type: "**joint_trajectory_controller/JointTrajectoryController**"
* Click "**Add Planning Group Joints**"

![Step 18](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup18.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".


![Step 19](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup19.png)

* Now you should see the added ros2 controller and corresponding joint in main ros2 controllers window

![Step 20](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup20.png)

#### Step 11: Set up ROS2 Controllers for Hand (Gripper Control)

* Click "**add controller**" 

![Step 20](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup20.png)

* Controller Name: "**hand_controller**"
* Controller Type: "**position_controllers/GripperActionController**"
* Click "**Add Planning Group Joints**"

![Step 21](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup21.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".

![Step 22](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup22.png)

* Now you should see the added ros2 controller and corresponding joint in main ros2 controllers window

![Step 23](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup23.png)

#### Step 12: Set up Moveit Controllers for Robot Arm

* Click "**add controller**" 

![Step 24](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup24.png)


* Controller Name: "**panda_arm_controller**"
* Controller Type: "**FollowJointTrajectory**"
* Action Namespace: "**follow_joint_trajectory**" 
* Default: true
* Click "**Add Planning Group Joints**"

![Step 25](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup25.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".

![Step 26](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup26.png)

#### Step 13: Set up Moveit Controllers for Robot Arm

* Click "**add controller**" 

![Step 26](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup26.png)

* Controller Name: "**hand_controller**"
* Controller Type: "**GripperCommand**"
* Action Namespace: "**gripper_cmd**" 
* Default: true
* Click "**Add Planning Group Joints**"

![Step 27](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup27.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".

![Step 28](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup28.png)

* Now you should see the added moveit controller and corresponding joint in main moveit controllers window

![Step 29](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup29.png)

#### Step 14: Skip Perception 3D Sensors

![Step 30](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup30.png)

#### Step 15 : Make sure all launch configuration files are selected

![Step 31](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup31.png)

#### Step 16 : Add Author Information

![Step 32](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup32.png)

#### Step 17: Generate Configuration Files

![Step 33](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup33.png)

* Select the location of the package folder and click generate package

![Step 34](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup34.png)

Build and verify the moveit package functionality with a demo

```
cd ~/day1_ws
colcon build --packages-select panda_moveit_config
source install/setup.bash
```

Launch Demo
```
ros2 launch panda_moveit_config demo.launch.py
```

You should see a simliar window as shown in the image below


![Step 35](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup35.png)

To do a simple demo, 

* Select "**panda_arm**" as Planning Group
* Select "**home**" as Start State
* Select "**random valid**" as goal state
* Click "**Plan & Execute**"

<br>

## ROS-I Manipulation Training Day 1 Exercise 3: Use Move Group and Move Group Interface to execute set of motion plan tasks

There are three objectives to this exercise

1) Pose Goal and Plan
2) Add Collision Object and Plan
3) Joint Goal and Plan


Let's first open the executable to be edited for this exercise

![Folder_img](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/moveit_exe_folder_structure.png)



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

![Pose Goal Template](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/pose_goal_template.png)

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

![Rviz](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/rviz_moveit.png)


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

![Add Collision Template](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/add_collision_template.png)


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

![Add Joint Goal Template](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/joint_goal_template.png)


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

![Rviz](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/rviz_moveit.png)


### Terminal 4: [Launch Moveit Interface: Calls Moveit group with motion plan and collision requests]

```
source ~/day1_ws/install/setup.bash
ros2 launch moveit_exec_pkg moveit_exec.launch.py
```

Now you should notice the robot planning (Note: not moving) from current joint position to target joint goal. 

<br>


## ROS-I Manipulation Training Day 1 Exercise 4: Visualize image (and) 3D pointcloud using camera plugin in gazebo and rviz

There are three objectives to this exercise

1) Add Camera Plugin in Gazebo Xacro file
2) Include Gazebo Xacro in URDF Xacro file
3) Visualize 3D Pointcloud and Image


Let's first open the executable to be edited for this exercise

![Folder_img](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/cam_sim_structure.png)


### Objective 1: Add Camera Plugin in Gazebo Xacro file

Open the gazebo xacro file to configure camera plugin

```
cd ~/day1_ws/src
code cam_sim_pkg/urdf/panda_gazebo.xacro
```
#### Gazebo Structure for Camera Plugin

![gazebo_struc](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/cam_sim_urdf_xacro.png)


Add the below gazebo tag to the template (panda_gazebo.xacro) based on above gazebo structure reference

```
    <!-- Add Camera Link name from URDF as gazebo reference -->
    <gazebo reference="mention the camera link name from URDF">
        <!-- Add Sensor section for Camera -->
        <!-- Add Sensor type as depth for 3D pointcloud or camera for 2D image -->
        <sensor type="mention_sensor_type" name="my_camera">
            <update_rate>20</update_rate>
            <visualize>true</visualize>
            <camera name="cam">
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>B8G8R8</format>
                </image>
                <clip>
                    <near>0.02</near>
                    <far>300</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                </noise>
            </camera>
            <!-- Add Camera Controller ROS gazbeo plugin for ROS interface -->
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <frame_name>camera_link</frame_name>
                <min_depth>0.1</min_depth>
                <max_depth>500</max_depth>
            </plugin>
        </sensor>
    </gazebo>
```

Add above tag to the below section in panda_gazebo.xacro code


![add_tag](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/gazebo_add_tag.png)


### Objective 2: Include Gazebo Xacro in URDF Xacro file

In this second objective, you are importing the gazebo xacro file from the main panda urdf file. 

Open the panda urdf xacro file (panda.urdf.xacro) to import panda_gazebo.xacro

```
cd ~/day1_ws/src
code cam_sim_pkg/urdf/panda.urdf.xacro
```

Edit the below section of the code with
  * Corresponding package name
  * Panda Gazebo Xacro File name


![include_tag](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/camera_link_include_gazebo_xacro.png)


### Objective 3.1: Visualize 3D Pointcloud

Now edit the gazebo Camera plugin to visualize pointcloud and image

```
cd ~/day1_ws/src
code cam_sim_pkg/urdf/panda_gazebo.xacro
```

To Visualize ***point cloud***, edit the following section of panda_gazebo.xacro
  * Mention the camera link name based on main urdf
  * Mention the sensor_type as "depth"

```
cd ~/day1_ws/
colcon build --packages-select cam_sim_pkg
source install/setup.bash
```


##### Terminal 1 [Launch Gazebo World: Gazebo world launches Robot Model and camera to simulate and visualize  pointcloud] 

```
source ~/day1_ws/install/setup.bash
ros2 launch cam_sim_pkg panda_cam_sim.launch.py
```

Now you should see the window similar to the image below

![gazebo](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/gazebo_depth.png)


##### Terminal 2 [Run Joint Commands: Publish Joint Commands to keep the robot arm in zero position] 

```
source ~/day1_ws/install/setup.bash
ros2 topic pub -r 100 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {frame_id: world}, joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, panda_finger_joint1, panda_finger_joint2], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}'
```

##### Terminal 3 [RViz: To visualize the pointcloud] 

```
source ~/day1_ws/install/setup.bash
rviz2
```

#### Steps to visualize

##### First Load Robot Model

Select "world' as fixed frame in RViz

![frame](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/world_frame_selection.png)

Load Robot Model in RViz window

Click "Add" in Displays Tab and select "RobotModel" from "By Display Type"

![toolselect](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/robotmodel_tool_selection.png)

Select "Description Topic" from added RobotModel

![topicselect](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/robot_desc_topic_selection.png)


##### Now Visualize Point Cloud

Click "Add" in Displays Tab and select "/points/PointCloud2" from "By Topic"

![pcselect](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/pointcloud_selection.png)

Now you should see the point cloud similar to the window below

![pointcloud](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/pointcloud_image.png)



### Objective 3.2: Visualize 2D Image


To Visualize ***image***, edit the following section of panda_gazebo.xacro
  * Mention the camera link name based on main urdf
  * Mention the sensor_type as "camera"

```
cd ~/day1_ws/
colcon build --packages-select cam_sim_pkg
source install/setup.bash
```


##### Terminal 1 [Launch Gazebo World: Gazebo world launches Robot Model and camera to simulate and visualize  pointcloud] 

```
source ~/day1_ws/install/setup.bash
ros2 launch cam_sim_pkg panda_cam_sim.launch.py
```

Now you should see the window similar to the image below

![gazebo](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/gazebo_camera.png)


##### Terminal 2 [Run Joint Commands: Publish Joint Commands to keep the robot arm in zero position] 

```
source ~/day1_ws/install/setup.bash
ros2 topic pub -r 100 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {frame_id: world}, joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, panda_finger_joint1, panda_finger_joint2], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}'
```

##### Terminal 3 [RViz: To visualize the pointcloud] 

```
source ~/day1_ws/install/setup.bash
rviz2
```

#### Steps to visualize

##### First Load Robot Model

Select "world' as fixed frame in RViz

![frame](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/world_frame_selection.png)

Load Robot Model in RViz window

Click "Add" in Displays Tab and select "RobotModel" from "By Display Type"

![toolselect](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/robotmodel_tool_selection.png)

Select "Description Topic" from added RobotModel

![topicselect](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/robot_desc_topic_selection.png)


##### Now Visualize 2D Image

Click "Add" in Displays Tab and select "/my_camera/image_raw/Image" from "By Topic"

![pcselect](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/image_selection.png)

Now you should see the window similar to the image below

![image](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/2d_image_rviz.png)






