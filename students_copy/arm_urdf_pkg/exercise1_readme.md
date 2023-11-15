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


