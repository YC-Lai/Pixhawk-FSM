#include "keyboard_ctrl.h"

#include <assert.h>

#include <cmath>
#include <iostream>

#include "util.h"

// using namespace std;

Keyboard_ctrl::Keyboard_ctrl(double height, double speed, int update_rate) : Ctrl(height) {
    shift = speed / update_rate;
    shift_angle = 0.8 * speed / update_rate;
    setpoint_height = takeoff_height;
}

// changes the Keyboard_ctrl speed once the Keyboard_ctrl is moving
void Keyboard_ctrl::ST_Move(std::shared_ptr<EventData> pData) {
    if (!is_executing_operation) {
        auto Data = std::static_pointer_cast<Keyboard_data>(pData);

        ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": "
                                                          << "Current state: MOVE");
        std::lock_guard<std::mutex> pose_guard(*(state_mutex_));
        geometry_msgs::Pose target_pose = current_pose.pose;
        auto current_ypr = Util::quaternion_to_euler_angle(current_pose.pose.orientation);

        switch (Data->input) {
            case MANEUVER::FORWARD:
                target_pose.position.x += std::cos(current_ypr.z) * shift;
                target_pose.position.y += std::sin(current_ypr.z) * shift;
                break;
            case MANEUVER::BACKWARD:
                target_pose.position.x -= std::cos(current_ypr.z) * shift;
                target_pose.position.y -= std::sin(current_ypr.z) * shift;
                break;
            case MANEUVER::LEFT:
                target_pose.position.x += std::cos(current_ypr.z + PI / 2.0) * shift;
                target_pose.position.y += std::sin(current_ypr.z + PI / 2.0) * shift;
                break;
            case MANEUVER::RIGHT:
                target_pose.position.x += std::cos(current_ypr.z - PI / 2.0) * shift;
                target_pose.position.y += std::sin(current_ypr.z - PI / 2.0) * shift;
                break;
            case MANEUVER::UP:
                setpoint_height += shift;
                break;
            case MANEUVER::DOWN:
                setpoint_height -= shift;
                break;
            case MANEUVER::TURNLEFT: {
                current_ypr.z += shift_angle;
                target_pose.orientation = Util::euler_to_quaternion(current_ypr);
                break;
            }
            case MANEUVER::TURNRIGHT: {
                current_ypr.z -= shift_angle;
                target_pose.orientation = Util::euler_to_quaternion(current_ypr);
                break;
            }
        }
        target_pose.position.z = setpoint_height;

        std::cout << "===Get target Point===" << std::endl;
        std::cout << "[Current pose]" << std::endl;
        std::cout << "x: " << current_pose.pose.position.x
                  << ", y: " << current_pose.pose.position.y
                  << ", z: " << current_pose.pose.position.z
                  << ", yaw: " << Util::quaternion_to_euler_angle(current_pose.pose.orientation).z
                  << std::endl;
        std::cout << "[Target pose]" << std::endl;
        std::cout << "x: " << target_pose.position.x << ", y: " << target_pose.position.y
                  << ", z: " << target_pose.position.z
                  << ", yaw: " << Util::quaternion_to_euler_angle(target_pose.orientation).z
                  << std::endl;
        std::cout << "======================" << std::endl;

        pixhawk_fsm::KB_Travel kb_travel_service_handle;
        kb_travel_service_handle.request.path = {target_pose};
        if (kb_travel.call(kb_travel_service_handle)) {
            if (!kb_travel_service_handle.response.success) {
                ROS_FATAL_STREAM(kb_travel_service_handle.response.message);
            } else {
                is_executing_operation = true;
            }
        } else {
            ROS_FATAL("Failed to call travel service.");
        }
    }
}