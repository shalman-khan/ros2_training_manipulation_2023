#!/bin/bash

# To run Base Script Follow the following steps
# Get to the path of the downloaded bash script file and type following two commands
# chmod +x manipulation_training_image_v1_update.sh
# ./manipulation_training_image_v1_update.sh
# The above two commands should update the version

# Prompt for sudo password at the beginning
echo "rosi" | sudo -S echo "Password Added for Permission"

# Update package lists
sudo apt update

# Install Visual Studio Code via Snap
sudo snap install code --classic

# Install Python package transforms3d
pip3 install transforms3d

# Install ROS packages
echo "y" | sudo apt-get install ros-humble-tf-transformations
echo "y" | sudo apt install ros-humble-librealsense2*
echo "y" | sudo apt install ros-humble-realsense2-*

echo "Version 1.1 Update done successfully ..."

