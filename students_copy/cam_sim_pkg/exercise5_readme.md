## ROS-I Manipulation Training Day 1 Exercise 5: Visualize image (and) 3D pointcloud using camera plugin in gazebo and rviz

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






