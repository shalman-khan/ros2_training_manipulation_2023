## ROS-I Training Day 4: Pick and Place Assessment

The objective is to scan three collision objects
1) Tray 1
2) Tray 2
3) Aruco Tagged Box

Aruco Tagged Box should be picked from Tray 1 and placed in Tray2


[![Watch the video](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day4/simulation_student_copy/misc_files/thumbnail.png)](https://drive.google.com/file/d/1-mHyQSyAIe6ky_drLt9frTdwkg85A5WK/view?usp=drive_link)



![readme_drawio](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day4/simulation_student_copy/misc_files/readme_day4.png)

### To verify spawned collision object [Use RVIZ and select pointcloud from topics]

![verification](https://github.com/shalman-khan/ros2_training_manipulation_2023/blob/day4/simulation_student_copy/misc_files/tray1_pointcloud_plus_spawn_aruco.png)


[Use terminal 1,2,3 alone for aruco detection client section]
[Use all for pick and place pipeline]

### Terminal 1 [Launch UR Simulation or real driver]

```
ros2 launch pick_and_place ur_bringup.launch.py
```

### Terminal 2 [Run rosbag][-l added for looping the rosbag]
```
ros2 bag play <tray1_rosbag> or <tray2_rosbag> or <tray1_rosbag_with_aruco_box> - l
```

### Terminal 3 [Run Aruco Detection node]
```
ros2 run ros2_aruco aruco_node
```

### Terminal 4 [Launch Gripper Driver]

```
ros2 launch gripper_driver_interface gripper_bringup.launch.py
```
### Terminal 4 [After Gripper Driver Launch, Launch Fake Service to control Gripper in same terminal]
```
ros2 launch robotiq_ros_service robotiq_ros_service.launch.py
```

### Terminal 5 [ Activate joint trajectory controller and launch pick and place pipeline]
https://control.ros.org/master/doc/ros2_control/ros2controlcli/doc/userdoc.html#switch-controllers
```
Activate Controller

ros2 launch pick_and_place pick_and_place.launch.py
```



