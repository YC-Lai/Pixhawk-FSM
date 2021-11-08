/**
 * @file travel_operation.h
 */

#ifndef TRAVEL_OPERATION_H
#define TRAVEL_OPERATION_H

#include "mavros_interface.h"
#include "move_operation.h"
#include "operation_identifier.h"

/**
 * @brief Represents the operation where the drone is moving quickly at large distances.
 */
class TravelOperation : public MoveOperation {
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
    TravelOperation(const std::vector<geometry_msgs::Pose>& path)
        : MoveOperation(OperationIdentifier::TRAVEL, path,
                        Pixhawk_fsm::getInstance().configuration.travel_speed, 5, 100, 0.1,
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