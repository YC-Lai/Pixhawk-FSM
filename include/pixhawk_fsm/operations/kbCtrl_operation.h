/**
 * @file move_operation.h
 */

#ifndef KBCTRL_OPERATION_H
#define KBCTRL_OPERATION_H

#include "mavros_interface.h"
#include "move_operation.h"
#include "operation_identifier.h"

/**
 * @brief Represents the operation where the drone is controlled by keyboard.
 */

class KbCtrlOperation : public MoveOperation {
   public:
    /**
     * @brief Sets up the travel operation.
     *
     * @param path List of setpoints.
     * @param speed is the travel speed in [m/s].
     * @param position_threshold means that setpoints count as visited within 2 [m].
     * @param 3 is the maximum speed the drone can have in the setpoint
     *          to mark it as visited [m/s].
     * @param max_angle is the maximum tilt angle of the drone during movement [deg].
     *                  This is set in the base.launch file.
     */
    KbCtrlOperation(const std::vector<geometry_msgs::Point>& path)
        : MoveOperation(OperationIdentifier::KEYBOARD, path,
                        Pixhawk_fsm::getInstance().configuration.travel_speed, 0.5, 1,
                        Pixhawk_fsm::getInstance().configuration.travel_max_angle) {
        MavrosInterface mavros_interface;
        mavros_interface.setParam("MPC_ACC_HOR",
                                  Pixhawk_fsm::getInstance().configuration.travel_accel * 100);
        ROS_INFO_STREAM(ros::this_node::getName().c_str()
                        << ": Set max acceleration to: "
                        << Pixhawk_fsm::getInstance().configuration.travel_accel << " m/s2.");
    }
};

#endif