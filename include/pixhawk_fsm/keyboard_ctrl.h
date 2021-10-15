#ifndef _KEYBOARD_CTRL_H
#define _KEYBOARD_CTRL_H
#include <pixhawk_fsm/Explore.h>
#include <pixhawk_fsm/Land.h>
#include <pixhawk_fsm/OperationCompletion.h>
#include <pixhawk_fsm/TakeOff.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <memory>
#include <mutex>

#include "state_machine.h"

// structure to hold event data passed into state machine
struct Keyboard_Data : public EventData {
    geometry_msgs::Point offset;
};

// the Keyboard_ctrl state machine class
class Keyboard_ctrl : public StateMachine {
   public:
    Keyboard_ctrl(double speed, int update_rate);

    // external events taken by this state machine
    void Land();
    void Move(std::shared_ptr<Keyboard_Data>);

   private:
    // ros module
    ros::NodeHandle node_handle;
    ros::Subscriber pose_subscriber;

    // Declared const as we only need to set up the service and get the calls to the callback, we
    // don't need to do anything with this object.
    ros::ServiceServer operation_completion_server;
    ros::ServiceClient take_off;
    ros::ServiceClient explore;
    ros::ServiceClient land;

    bool is_executing_operation = false;
    std::string finished_operation = "";

    /**
     * @brief Current pose.
     */
    geometry_msgs::Point current_pose;

    /**
     * @brief constant speed ardupilot parameter for the keyboard control.
     */
    double move_speed;

    int update_rate;

    std::unique_ptr<std::mutex> pose_mutex_;

    /**
     * @brief Retrieves the pose of the drone.
     *
     * @param pose_ptr Current pose.
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr);

    /**
     * @brief Grab the finished operation when it completes. This is just a service callback that
     * pixhawk_fsm will call to. It's not mandatory to implement this service callback, the result of the
     * callback does not affect the internal state of pixhawk_fsm, but in that case the client will of
     * course not know when an operation completes. In other words, just leave this here to have a
     * two-way communication :)
     *
     * @param request Holds the operation completed.
     * @param response Just an empty struct (not used, only required as a parameter for the ROS
     * service).
     *
     * @return true as this service will always succeed, since the pixhawk_fsm server don't care if this
     * service fails or not it's just a measure for the two way communication.
     */
    bool OperationCompletionCallback(pixhawk_fsm::OperationCompletion::Request& request,
                                          pixhawk_fsm::OperationCompletion::Response& response);

    /**
     * @brief Waits for @p timeout until we get connection with the services.
     *
     * @param timeout Time to wait for each service.
     *
     * @return true if we got connetion with all the services.
     */
    bool gotConnectionWithServices(const unsigned int& timeout);

    // state machine state functions
    void ST_Idle(EventData*);
    void ST_Land(EventData*);
    void ST_Takeoff(std::shared_ptr<Keyboard_Data>);
    void ST_Move(std::shared_ptr<Keyboard_Data>);

    // state map to define state function order
    BEGIN_STATE_MAP
    STATE_MAP_ENTRY(&Keyboard_ctrl::ST_Idle)
    STATE_MAP_ENTRY(&Keyboard_ctrl::ST_Land)
    STATE_MAP_ENTRY(&Keyboard_ctrl::ST_Takeoff)
    STATE_MAP_ENTRY(&Keyboard_ctrl::ST_Move)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States { ST_IDLE = 0, ST_LAND, ST_TAKEOFF, ST_MOVE, ST_MAX_STATES };
};
#endif  // _KEYBOARD_CTRL_H