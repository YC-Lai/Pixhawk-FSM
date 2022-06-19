# Pixhawk FSM
A control system created through FSM design pattern.  
The following fig is an experiment which testing on custom Pixhawk drone in the real world.

![cover](fig/test.gif)

Work with SLAM:  
[![SLAM](http://img.youtube.com/vi/SCfZ7u6d8cw/0.jpg)](http://www.youtube.com/watch?v=SCfZ7u6d8cw)


## Dependencies for Running Locally
1. [Ros Melodic](http://wiki.ros.org/melodic/Installation) 
2. [MAVROS](https://github.com/mavlink/mavros)
3. [ESES](https://github.com/HKPolyU-UAV/E2ES.git)  
I utilize the Gazobo drone model create by this repository for simulation. ( you can skip this step if you have your own model. )

## Installation
Ubuntu 18.04 (ROS melodic) were tested.  

    cd ~/catkin_ws/src
    git clone https://github.com/YC-Lai/Pixhawk-FSM.git
    catkin build pixhawk_fsm

## Usage
- Run the **simulator** and use keyboard to control the drone:
    ```
    roscd pixhawk_fsm
    ./scripts/px4_stil.sh
    roslaunch pixhawk_fsm simulator.launch
    rosrun pixhawk_fsm keyboard_client
    ```
- Using keyboard to control the MAV in **real world**:
    ```
    rosrun pixhawk_fsm keyboard_client
    ```

## TODO
- [ ] add path planning agent
- [ ] extend to avaliable SLAM system

## License
Pixhawk FSM is a public domain work. Feel free to do whatever you want with it.
