#!/bin/bash

# px4 code and mavros
cd ~/catkin_ws/src
git clone https://github.com/YC-Lai/Pixhawk-FSM.git
cd Pixhawk_FSM
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
sudo chmod +x ./resource/install_geographic.sh
sudo usermod -a -G dialout ${USER}
sudo usermod -a -G tty ${USER}

echo "You might have to reboot to use mavros"
