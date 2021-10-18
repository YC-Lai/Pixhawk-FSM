#include "keyboard_ctrl.h"

#include <assert.h>

#include <iostream>

#include "util.h"

// using namespace std;

Keyboard_ctrl::Keyboard_ctrl(double speed, int update_rate)
    : StateMachine(ST_MAX_STATES), move_speed(speed), update_rate(update_rate), path_density(4) {
    pose_subscriber =
        node_handle.subscribe("mavros/local_position/pose", 1, &Keyboard_ctrl::poseCallback, this);
    operation_completion_server = node_handle.advertiseService(
        "pixhawk_fsm/operation_completion", &Keyboard_ctrl::OperationCompletionCallback, this);
    take_off = node_handle.serviceClient<pixhawk_fsm::TakeOff>("pixhawk_fsm/take_off");
    explore = node_handle.serviceClient<pixhawk_fsm::Explore>("pixhawk_fsm/explore");
    land = node_handle.serviceClient<pixhawk_fsm::Land>("pixhawk_fsm/land");

    if (!gotConnectionWithServices(2)) {
        ROS_FATAL("Did not get connection with pixhawk_fsm's services, is pixhawk_fsm running?");
        throw std::runtime_error("Did not get connection with pixhawk_fsm's services");
    }

    pose_mutex_.reset(new std::mutex);
}

void Keyboard_ctrl::poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr) {
    std::lock_guard<std::mutex> pose_guard(*(pose_mutex_));
    current_pose.x = pose_ptr->pose.position.x;
    current_pose.y = pose_ptr->pose.position.y;
    current_pose.z = pose_ptr->pose.position.z;
}

bool Keyboard_ctrl::OperationCompletionCallback(
    pixhawk_fsm::OperationCompletion::Request& request,
    pixhawk_fsm::OperationCompletion::Response& response) {
    finished_operation = request.operation;
    is_executing_operation = false;

    return true;
}

bool Keyboard_ctrl::gotConnectionWithServices(const unsigned int& timeout) {
    if (!ros::service::waitForService("pixhawk_fsm/take_off", timeout)) {
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
void Keyboard_ctrl::Land(void) {
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
void Keyboard_ctrl::Move(std::shared_ptr<Keyboard_Data> pData) {
    BEGIN_TRANSITION_MAP                     // - Current State -
    TRANSITION_MAP_ENTRY(ST_TAKEOFF)         // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(ST_MOVE)        // ST_Takeoff
        TRANSITION_MAP_ENTRY(ST_MOVE)        // ST_Move
        END_TRANSITION_MAP(pData)
}

// state machine sits here when Keyboard_ctrl is not running
void Keyboard_ctrl::ST_Idle(EventData* pData) { ROS_INFO_STREAM("[Client]: Current state: IDLE"); }

// stop the Keyboard_ctrl
void Keyboard_ctrl::ST_Land(EventData* pData) {
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

    // perform the stop Keyboard_ctrl processing here
    // transition to ST_Idle via an internal event
    InternalEvent(ST_IDLE);
}

// start the Keyboard_ctrl going
void Keyboard_ctrl::ST_Takeoff(std::shared_ptr<Keyboard_Data> pData) {
    ROS_INFO_STREAM("[Client]: Current state: TAKEOFF");
    // set initial Keyboard_ctrl speed processing here
    pixhawk_fsm::TakeOff take_off_service_handle;
    take_off_service_handle.request.height = 1.5f;
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

// changes the Keyboard_ctrl speed once the Keyboard_ctrl is moving
void Keyboard_ctrl::ST_Move(std::shared_ptr<Keyboard_Data> pData) {
    ROS_INFO_STREAM("[Client]: Current state: MOVE");

    // perform the change Keyboard_ctrl speed to pData->speed here
    geometry_msgs::Point target_point;
    geometry_msgs::Point offset = pData->offset;
    std::lock_guard<std::mutex> pose_guard(*(pose_mutex_));
    target_point.x = current_pose.x + (offset.x * move_speed / update_rate);
    target_point.y = current_pose.y + (offset.y * move_speed / update_rate);
    target_point.z = current_pose.z + (offset.z * move_speed / update_rate);
    std::cout << "===Get target Point===" << std::endl;
    std::cout << "Current pose" << std::endl;
    std::cout << "x: " << current_pose.x << ", y: " << current_pose.y << ", z: " << current_pose.z
              << std::endl;
    std::cout << "Target pose" << std::endl;
    std::cout << "x: " << target_point.x << ", y: " << target_point.y << ", z: " << target_point.z
              << std::endl;

    std::vector<geometry_msgs::Point> path =
        Util::createPath(current_pose, target_point, path_density);

    pixhawk_fsm::Explore explore_service_handle;
    explore_service_handle.request.path = path;
    explore_service_handle.request.point_of_interest = target_point;
    if (explore.call(explore_service_handle)) {
        if (!explore_service_handle.response.success) {
            ROS_FATAL_STREAM(explore_service_handle.response.message);
        } else {
            is_executing_operation = true;
        }
    } else {
        ROS_FATAL("Failed to call explore service.");
    }
}