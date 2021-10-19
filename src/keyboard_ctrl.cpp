#include "keyboard_ctrl.h"

#include <assert.h>

#include <cmath>
#include <iostream>

#include "util.h"

// using namespace std;

Keyboard_ctrl::Keyboard_ctrl(double height, double speed, int update_rate)
    : Ctrl(height), move_speed(speed), update_rate(update_rate), path_density(2) {;}

// changes the Keyboard_ctrl speed once the Keyboard_ctrl is moving
void Keyboard_ctrl::ST_Move(std::shared_ptr<Keyboard_data> pData) {
    ROS_INFO_STREAM("[Client]: Current state: MOVE");
    std::lock_guard<std::mutex> pose_guard(*(state_mutex_));
    geometry_msgs::Point target_point = current_pose.pose.position;
    switch (pData->input) {
        case MANEUVER::FORWARD:
            target_point.x +=
                std::cos(Util::quaternion_to_euler_angle(current_pose.pose.orientation).x) *
                (move_speed / update_rate);
            target_point.y +=
                std::sin(Util::quaternion_to_euler_angle(current_pose.pose.orientation).y) *
                (move_speed / update_rate);
            break;
        case MANEUVER::BACKWARD:
            target_point.x -=
                std::cos(Util::quaternion_to_euler_angle(current_pose.pose.orientation).x) *
                (move_speed / update_rate);
            target_point.y -=
                std::sin(Util::quaternion_to_euler_angle(current_pose.pose.orientation).y) *
                (move_speed / update_rate);
            break;
        case MANEUVER::LEFT:
            target_point.x +=
                std::cos(Util::quaternion_to_euler_angle(current_pose.pose.orientation).x +
                         PI / 2.0) *
                (move_speed / update_rate);
            target_point.y +=
                std::sin(Util::quaternion_to_euler_angle(current_pose.pose.orientation).y +
                         PI / 2.0) *
                (move_speed / update_rate);
            break;
        case MANEUVER::RIGHT:
            target_point.x +=
                std::cos(Util::quaternion_to_euler_angle(current_pose.pose.orientation).x -
                         PI / 2.0) *
                (move_speed / update_rate);
            target_point.y +=
                std::sin(Util::quaternion_to_euler_angle(current_pose.pose.orientation).y -
                         PI / 2.0) *
                (move_speed / update_rate);
            break;
        case MANEUVER::UP:
            target_point.z += move_speed / update_rate;
            break;
        case MANEUVER::DOWN:
            target_point.z -= move_speed / update_rate;
            break;
    }

    std::cout << "===Get target Point===" << std::endl;
    std::cout << "Current pose" << std::endl;
    std::cout << "x: " << current_pose.pose.position.x << ", y: " << current_pose.pose.position.y
              << ", z: " << current_pose.pose.position.z << std::endl;
    std::cout << "Target pose" << std::endl;
    std::cout << "x: " << target_point.x << ", y: " << target_point.y << ", z: " << target_point.z
              << std::endl;

    std::vector<geometry_msgs::Point> path =
        Util::createPath(current_pose.pose.position, target_point, path_density);

    pixhawk_fsm::Travel travel_service_handle;
    travel_service_handle.request.path = path;
    if (travel.call(travel_service_handle)) {
        if (!travel_service_handle.response.success) {
            ROS_FATAL_STREAM(travel_service_handle.response.message);
        } else {
            is_executing_operation = true;
        }
    } else {
        ROS_FATAL("Failed to call travel service.");
    }
}