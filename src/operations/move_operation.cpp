/**
 * @file move_operation.cpp
 */

#include "move_operation.h"

#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/ParamSet.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include "mavros_interface.h"
#include "pixhawk_fsm.h"
#include "util.h"

MoveOperation::MoveOperation(const OperationIdentifier& operation_identifier,
                             const std::vector<geometry_msgs::Pose>& path, const double& speed,
                             const double& position_threshold, const double& velocity_threshold,
                             const double& yaw_threshold, const double& max_angle = 45)
    : Operation(operation_identifier, false, true),
      path(path),
      speed(speed * 100),
      position_threshold(position_threshold),
      velocity_threshold(velocity_threshold),
      yaw_threshold(yaw_threshold),
      max_angle(max_angle * 100) {}

bool MoveOperation::hasFinishedExecution() const { return been_to_all_points; }

void MoveOperation::initialize() {
    for (auto iterator = path.begin(); iterator != path.end(); iterator++) {
        if (iterator->position.z <= 0.1) {
            iterator->position.z = Pixhawk_fsm::getInstance().configuration.default_height;
        }
    }

    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;
    been_to_all_points = false;
    current_setpoint_iterator = path.begin();
    setpoint.position = current_setpoint_iterator->position;
    setpoint.yaw = Util::quaternion_to_euler_angle(current_setpoint_iterator->orientation).z;

    if (identifier != OperationIdentifier::KEYBOARD) {
        double dx = current_setpoint_iterator->position.x - getCurrentPose().pose.position.x;
        double dy = current_setpoint_iterator->position.y - getCurrentPose().pose.position.y;
        setpoint.yaw = std::atan2(dy, dx);
    }

    MavrosInterface mavros_interface;
    mavros_interface.setParam("MPC_XY_VEL_ALL", speed);
    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                    << ": Set speed to: " << speed / 100 << " m/s.");

    mavros_interface.setParam("MPC_MAN_TILT_MAX", max_angle);
    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                    << ": Set max angle to: " << max_angle / 100 << " deg.");

    time_start = clock();
}

void MoveOperation::tick() {
    bool at_position_target =
        Util::distanceBetween(getCurrentPose().pose.position, current_setpoint_iterator->position) <
        position_threshold;
    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;
    bool at_yaw_target = Util::yawBetween(getCurrentPose().pose.orientation,
                                          current_setpoint_iterator->orientation) < yaw_threshold;

    time_stop = clock();
    auto time_duration = double(time_stop - time_start) / CLOCKS_PER_SEC;
    // std::cout << "time: " << time_duration << std::endl;
    if (time_duration >= 0.02 && low_enough_velocity) {
        update_setpoint = true;
    }

    if ((at_position_target && low_enough_velocity && at_yaw_target) || update_setpoint) {
        if (current_setpoint_iterator < path.end() - 1) {
            current_setpoint_iterator++;

            setpoint.position = current_setpoint_iterator->position;
            setpoint.yaw =
                Util::quaternion_to_euler_angle(current_setpoint_iterator->orientation).z;

            std::cout << "===Update Setpoint Point===" << std::endl;
            std::cout << "x: " << setpoint.position.x << ", y: " << setpoint.position.y
                      << ", z: " << setpoint.position.z << std::endl;

            if (identifier != OperationIdentifier::KEYBOARD) {
                double dx =
                    current_setpoint_iterator->position.x - getCurrentPose().pose.position.x;
                double dy =
                    current_setpoint_iterator->position.y - getCurrentPose().pose.position.y;
                setpoint.yaw = std::atan2(dy, dx);
            }

        } else {
            been_to_all_points = true;
        }

        update_setpoint = false;
    }
}
