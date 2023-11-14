## ROS-I Manipulation Training Day 1 Exercise 2: Use Moveit Setup Assistant to Develop Moveit Config Package


For this exercise, 
1) **Moveit Setup Assistant** will be used to create Moveit Config

```
cd ~/day1_ws
ros2 launch moveit_setup_assistant setup_assistant.launch.py 
```

#### Step 1: Click ***"Create New Moveit Configuration Package"*** and select and load the created URDF file "panda.urdf" from first excercise. 

![Step 1](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup1.png)

Now you should see a robot model loaded at the right display window

![Step 2](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup2.png)


#### Step 2: Click Generate Collision Matric

![Step 3](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup3.png)

Now you should see a list of link pairs with enable/disable collision and reasons to disable collision

![Step 4](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup4.png)



#### Step 3: Define Robot Arm Planning Group
* Click "**add group**" 
* Goup Name: "**panda_arm**"
* Kinematic Solver: "**KDL Kinematics Plugin**"
* Group Default Planner: None 
* Click "**Add Joints**"

![Step 5](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup5.png)

* Now you should see list of joint names and select the highlighted group of joints and press "**>**" button.
* Once done, click "**save**".

![Step 6](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup6.png)

* Now you should see the added planning group with selected joints in main planning group window

![Step 7](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup7.png)

#### Step 4: Define Robot Arm Planning Group
* Click "**add group**" 
* Goup Name: "**hand**"
* Kinematic Solver: "**None**"
* Group Default Planner: None 
* Click "**Add Links**"

![Step 8](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup8.png)

* Now you should see list of link names and select the highlighted group of links and press "**>**" button.
* Once done, click "**save**".

![Step 9](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup9.png)

* Now you should see the added planning group with selected links in main planning group window

![Step 10](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup10.png)

#### Step 5: Define Robot Home Pose

* Name pose as "**home**"
* Select "**panda_arm**" group 
* Select a random valid pose using sliders to set home position.


![Step 11](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup11.png)


#### Step 6: Define Gripper Pose

* Name pose as "**open**"
* Select "**hand**" group 
* Select a random valid pose using sliders to set gripper open position [prefer: 0.0100]

![Step 12](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup12.png)

* Name pose as "**close**"**
* Select "**hand**" group 
* Select a random valid pose using sliders to set gripper closed position [prefer: 0.0]

![Step 13](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup13.png)

#### Step 7: Set End Effectors

* Name End Effector as "**hand**"
* Select "**hand**" as end effector group 
* Select "**panda_link8**" as parent link
* leave parent group blank

![Step 14](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup14.png)

#### Step 8: Skip Passive Joints selection

![Step 15](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup15.png)

#### Step 9: ROS2 Control URDF Modification

* Select position as command interface
* Select position and velocity as state interface

![Step 16](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup16.png)


#### Step 10: Set up ROS2 Controllers for Robot Arm

* Click "**add controller**" 

![Step 17](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup17.png)

* Controller Name: "**panda_arm_controller**"
* Controller Type: "**joint_trajectory_controller/JointTrajectoryController**"
* Click "**Add Planning Group Joints**"

![Step 18](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup18.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".


![Step 19](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup19.png)

* Now you should see the added ros2 controller and corresponding joint in main ros2 controllers window

![Step 20](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup20.png)

#### Step 11: Set up ROS2 Controllers for Hand (Gripper Control)

* Click "**add controller**" 

![Step 20](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup20.png)

* Controller Name: "**hand_controller**"
* Controller Type: "**position_controllers/GripperActionController**"
* Click "**Add Planning Group Joints**"

![Step 21](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup21.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".

![Step 22](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup22.png)

* Now you should see the added ros2 controller and corresponding joint in main ros2 controllers window

![Step 23](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup23.png)

#### Step 12: Set up Moveit Controllers for Robot Arm

* Click "**add controller**" 

![Step 24](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup24.png)


* Controller Name: "**panda_arm_controller**"
* Controller Type: "**FollowJointTrajectory**"
* Action Namespace: "**follow_joint_trajectory**" 
* Default: true
* Click "**Add Planning Group Joints**"

![Step 25](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup25.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".

![Step 26](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup26.png)

#### Step 13: Set up Moveit Controllers for Robot Arm

* Click "**add controller**" 

![Step 26](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup26.png)

* Controller Name: "**hand_controller**"
* Controller Type: "**GripperCommand**"
* Action Namespace: "**gripper_cmd**" 
* Default: true
* Click "**Add Planning Group Joints**"

![Step 27](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup27.png)

* Now you should see list of group names and select the highlighted group and press "**>**" button.
* Once done, click "**save**".

![Step 28](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup28.png)

* Now you should see the added moveit controller and corresponding joint in main moveit controllers window

![Step 29](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup29.png)

#### Step 14: Skip Perception 3D Sensors

![Step 30](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup30.png)

#### Step 15 : Make sure all launch configuration files are selected

![Step 31](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup31.png)

#### Step 16 : Add Author Information

![Step 32](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup32.png)

#### Step 17: Generate Configuration Files

![Step 33](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup33.png)

* Select the location of the package folder and click generate package

![Step 34](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup34.png)

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


![Step 35](https://github.com/shalman-khan/ros2_manipulation_training_2023/blob/day1/students_copy/misc_files/moveit_setup/moveit_setup35.png)

To do a simple demo, 

* Select "**panda_arm**" as Planning Group
* Select "**home**" as Start State
* Select "**random valid**" as goal state
* Click "**Plan & Execute**"

