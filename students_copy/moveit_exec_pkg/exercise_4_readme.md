## ROS-I Manipulation Training Day 1 Exercise 3: ARUCO Detection and Spawn Collision Object


There are three objectives to this exercise

1) Write a client request funcion to get Aruco Marker Position
2) Call the detect_collision_object to get Marker pose
3) Create a box for following dimensions and use the detected pose as box pose


Let's first open the executable to be edited for this exercise

![Folder_img](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/ex4_folder_struc.png)


### For this exercise you will be provided with a logitech C270 camera
1) Restart your Virtual Machine
2) Add Logitech C270 using settings->usb

### Objective 1:  Write a client request funcion to get Aruco Marker Position

![obj1](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/Ex4_1.png)


### Objective 2:  Call the detect_collision_object to get Marker pose

![obj1](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/Ex4_2.png)

### Objective 3: Create a box for following dimensions and use the detected pose as box pose

![obj1](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/Ex4_3.png)


### Terminal 1: [Launch Camera]
```
cd ~/day1_ws
colcon build --packages-select moveit_exec_pkg
source install/setup.bash
ros2 launch cam_launch_pkg cam.launch.py 
```

### Terminal 2: [Launch Moveit Config with Controller Manager]

```
source ~/day1_ws/install/setup.bash
ros2 launch panda_moveit_config demo.launch.py
```

### Terminal 3: [Launch Moveit Interface to Spawn Collision Object along with Aruco Node and Camera TF]

```
source ~/day1_ws/install/setup.bash
ros2 launch moveit_exec_pkg moveit_spawn_collision.launch.py
```

Place the camera in front of the aruco Marker

![aruco](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/aruco_marker.png)


### Exercise 4: End Result

![exend](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day1/students_copy/misc_files/ex_end.png)



