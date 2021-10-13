/**
 * @file mavros_inerface.h
 */

#ifndef MAVROS_INTERFACE_H
#define MAVROS_INTERFACE_H

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

/**
 * @brief Handles communication regarding setting state, retriving state from the pixhawk, as well
 * as convenience functions for arming, offboard mode and setting parameters.
 */
class MavrosInterface {
   private:
    /**
     * @brief How fast the mavros interface will check for state changes when setting new modes in
     * Pixhawk.
     */
    const unsigned int UPDATE_REFRESH_RATE = 5;

    /**
     * @brief Retrieves the state changes within Pixhawk.
     */
    ros::Subscriber state_subscriber;

    /**
     * @brief Current state of Pixhawk.
     */
    mavros_msgs::State current_state;

    /**
     * @brief Publishes setpoints.
     */
    ros::Publisher setpoint_publisher;

    /**
     * @brief Callback for the state within Pixhawk.
     *
     * @param msg The state message.
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

   public:
    /**
     * @brief Sets up the required subscribers and service clients.
     */
    MavrosInterface();

    /**
     * @return The current state gotten from Pixhawk through mavros.
     */
    mavros_msgs::State getCurrentState() const;

    /**
     * @brief Sets up the connection with Pixhawk through MAVROS.
     */
    void establishContactToPixhawk() const;

    /**
     * @brief Will attempt to set the @p mode if Pixhawk is not already in the given mode.
     *
     * @param mode The mode to attempt to set.
     *
     * @return true if the mode got set.
     */
    bool attemptToSetMode(const std::string& mode) const;

    /**
     * @brief Requests Pixhawk to arm.
     *
     * @param auto_arm Will arm automatically if set to true, if not it'll wait until an arm signal
     * is retrieved from the RC.
     */
    void requestArm(const bool& auto_arm) const;

    /**
     * @brief Requests Pixhawk to go into offboard mode.
     *
     * @param auto_offboard Will go into offboard mode automatically if set to true, if not it'll
     * wait until offboard flight mode is set from RC.
     */
    void requestOffboard(const bool& auto_offboard) const;

    /**
     * @brief Requests Pixhawk to take off.
     *
     * @param height Will set the height we want to take off
     */
    void requestTakeOff(mavros_msgs::PositionTarget height) const;

    /**
     * @brief Sets a parameter within Pixhawk.
     *
     * @param parameter The parameter to set.
     * @param value The new value.
     */
    void setParam(const std::string& parameter, const float& value) const;
};

#endif
