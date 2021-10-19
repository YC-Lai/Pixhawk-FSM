#ifndef _CTRL_H
#define _CTRL_H
#include <geometry_msgs/PoseStamped.h>
#include <pixhawk_fsm/Explore.h>
#include <pixhawk_fsm/Land.h>
#include <pixhawk_fsm/OperationCompletion.h>
#include <pixhawk_fsm/TakeOff.h>
#include <pixhawk_fsm/Travel.h>
#include <ros/ros.h>

#include <memory>
#include <mutex>

#include "state_machine.h"

class Ctrl : public StateMachine {
   public:
    Ctrl(double height);

    // external events taken by this state machine
    void Land();
    void Move(std::shared_ptr<EventData> pData);

   protected:
    // ros module
    ros::NodeHandle node_handle;
    ros::Subscriber pose_subscriber;

    // Declared const as we only need to set up the service and get the calls to the callback, we
    // don't need to do anything with this object.
    ros::ServiceServer operation_completion_server;
    ros::ServiceClient take_off;
    ros::ServiceClient travel;
    ros::ServiceClient kb_travel;
    ros::ServiceClient explore;
    ros::ServiceClient land;

    bool is_executing_operation = false;
    std::string finished_operation = "";

    const double takeoff_height;

    /**
     * @brief Current pose.
     */
    geometry_msgs::PoseStamped current_pose;

    std::unique_ptr<std::mutex> state_mutex_;

    /**
     * @brief Retrieves the pose of the drone.
     *
     * @param pose_ptr Current pose.
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr);

    /**
     * @brief Grab the finished operation when it completes. This is just a service callback that
     * pixhawk_fsm will call to. It's not mandatory to implement this service callback, the result
     * of the callback does not affect the internal state of pixhawk_fsm, but in that case the
     * client will of course not know when an operation completes. In other words, just leave this
     * here to have a two-way communication :)
     *
     * @param request Holds the operation completed.
     * @param response Just an empty struct (not used, only required as a parameter for the ROS
     * service).
     *
     * @return true as this service will always succeed, since the pixhawk_fsm server don't care if
     * this service fails or not it's just a measure for the two way communication.
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
    void ST_Takeoff(std::shared_ptr<EventData>);
    virtual void ST_Move(std::shared_ptr<EventData>) {}

    // state map to define state function order
    BEGIN_STATE_MAP
    STATE_MAP_ENTRY(&Ctrl::ST_Idle)
    STATE_MAP_ENTRY(&Ctrl::ST_Land)
    STATE_MAP_ENTRY(&Ctrl::ST_Takeoff)
    STATE_MAP_ENTRY(&Ctrl::ST_Move)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States { ST_IDLE = 0, ST_LAND, ST_TAKEOFF, ST_MOVE, ST_MAX_STATES };    
};
#endif  // _CTRL_H