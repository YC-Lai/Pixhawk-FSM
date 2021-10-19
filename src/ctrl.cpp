#include "ctrl.h"

#include <iostream>

#include "util.h"

Ctrl::Ctrl(double height) : StateMachine(ST_MAX_STATES), takeoff_height(height) {
    pose_subscriber =
        node_handle.subscribe("mavros/local_position/pose", 1, &Ctrl::poseCallback, this);
    operation_completion_server = node_handle.advertiseService(
        "pixhawk_fsm/operation_completion", &Ctrl::OperationCompletionCallback, this);
    take_off = node_handle.serviceClient<pixhawk_fsm::TakeOff>("pixhawk_fsm/take_off");
    travel = node_handle.serviceClient<pixhawk_fsm::Travel>("pixhawk_fsm/travel");
    explore = node_handle.serviceClient<pixhawk_fsm::Explore>("pixhawk_fsm/explore");
    land = node_handle.serviceClient<pixhawk_fsm::Land>("pixhawk_fsm/land");

    if (!gotConnectionWithServices(2)) {
        ROS_FATAL("Did not get connection with pixhawk_fsm's services, is pixhawk_fsm running?");
        throw std::runtime_error("Did not get connection with pixhawk_fsm's services");
    }

    state_mutex_.reset(new std::mutex);
}

void Ctrl::poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr) {
    std::lock_guard<std::mutex> pose_guard(*(state_mutex_));
    current_pose = *(pose_ptr);
}

bool Ctrl::OperationCompletionCallback(pixhawk_fsm::OperationCompletion::Request& request,
                                       pixhawk_fsm::OperationCompletion::Response& response) {
    finished_operation = request.operation;
    is_executing_operation = false;

    return true;
}

bool Ctrl::gotConnectionWithServices(const unsigned int& timeout) {
    if (!ros::service::waitForService("pixhawk_fsm/take_off", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("pixhawk_fsm/travel", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("pixhawk_fsm/explore", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("pixhawk_fsm/land", timeout)) {
        return false;
    }

    return true;
}

// Land drone external event
void Ctrl::Land(void) {
    // given the Halt event, transition to a new state based upon
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                     // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)      // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Takeoff
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Move
        END_TRANSITION_MAP(NULL)
}

// move drone external event
void Ctrl::Move(std::shared_ptr<EventData> pData) {
    BEGIN_TRANSITION_MAP                     // - Current State -
    TRANSITION_MAP_ENTRY(ST_TAKEOFF)         // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(ST_MOVE)        // ST_Takeoff
        TRANSITION_MAP_ENTRY(ST_MOVE)        // ST_Move
        END_TRANSITION_MAP(pData)
}

// state machine sits here when Ctrl is not running
void Ctrl::ST_Idle(EventData* pData) { ROS_INFO_STREAM("[Client]: Current state: IDLE"); }

// stop the Ctrl
void Ctrl::ST_Land(EventData* pData) {
    ROS_INFO_STREAM("[Client]: Current state: LAND");

    pixhawk_fsm::Land land_service_handle;
    if (land.call(land_service_handle)) {
        if (!land_service_handle.response.success) {
            ROS_FATAL_STREAM(land_service_handle.response.message);
        } else {
            ROS_INFO_STREAM("[Client]: LAND success");
            is_executing_operation = true;
        }
    } else {
        ROS_FATAL("Failed to call land service.");
    }

    // perform the stop Ctrl processing here
    // transition to ST_Idle via an internal event
    InternalEvent(ST_IDLE);
}

// start the Ctrl going
void Ctrl::ST_Takeoff(std::shared_ptr<EventData> pData) {
    ROS_INFO_STREAM("[Client]: Current state: TAKEOFF");

    pixhawk_fsm::TakeOff take_off_service_handle;
    take_off_service_handle.request.height = takeoff_height;
    if (take_off.call(take_off_service_handle)) {
        if (!take_off_service_handle.response.success) {
            ROS_FATAL_STREAM(take_off_service_handle.response.message);
        } else {
            is_executing_operation = true;
        }
    } else {
        ROS_FATAL("Failed to call take off service.");
    }
}