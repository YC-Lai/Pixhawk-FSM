# Pixhawk FSM

## Installation
Ubuntu 18.04 (ROS melodic) were tested. This package can be either installed by [.sh file](https://github.com/YC-Lai/Pixhawk-FSM/blob/main/scripts/install.sh), or step-by-step manual installation.

### What you need
1. [Ros Melodic](http://wiki.ros.org/melodic/Installation) 
2. [MAVROS](https://github.com/mavlink/mavros)
3. [ESES](https://github.com/HKPolyU-UAV/E2ES.git)  
We utilize the Gazobo drone model create by this repository for simulation. ( you can skip this step if you have your own model. )
4. Pixhawk FSM
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/YC-Lai/Pixhawk-FSM.git
    catkin build pixhawk_fsm
    ```


## Usage
- Run the simulator and use keyboard to control the drone
    ```
    roscd pixhawk_fsm
    ./scripts/px4_stil.sh
    rosrun pixhawk_fsm keyboard_client
    ```
- Using keyboard to control the MAV in real world
    ```
    rosrun pixhawk_fsm keyboard_client
    ```
