#include "pixhawk_fsm.h"

#include <assert.h>

#include <iostream>

#include "hold_operation.h"
#include "mavros_interface.h"
#include "take_off_operation.h"

/******************************************************************************************************
 *                                          Singleton *
 ******************************************************************************************************/

std::shared_ptr<Pixhawk_fsm> Pixhawk_fsm::instance_ptr;

void Pixhawk_fsm::initialize(const PixhawkConfiguration configuration) {
    if (!instance_ptr) {
        // Can't use std::make_shared here as the constructor is private.
        instance_ptr = std::shared_ptr<Pixhawk_fsm>(new Pixhawk_fsm(configuration));
    }
}

Pixhawk_fsm& Pixhawk_fsm::getInstance() { return *instance_ptr; }

std::shared_ptr<StatusPublisher> Pixhawk_fsm::getStatusPublisherPtr() {
    return status_publisher_ptr;
}

Pixhawk_fsm::Pixhawk_fsm(const PixhawkConfiguration configuration)
    : StateMachine(ST_MAX_STATES), configuration(configuration) {
    take_off_server =
        node_handle.advertiseService("pixhawk_fsm/take_off", &Pixhawk_fsm::take_off, this);
    explore_server =
        node_handle.advertiseService("pixhawk_fsm/explore", &Pixhawk_fsm::explore, this);
    land_server = node_handle.advertiseService("pixhawk_fsm/land", &Pixhawk_fsm::land, this);
    operation_completion_client = node_handle.serviceClient<pixhawk_fsm::OperationCompletion>(
        "pixhawk_fsm/operation_completion");
    status_publisher_ptr = std::make_shared<StatusPublisher>();
}

/******************************************************************************************************
 *                                          Operations *
 ******************************************************************************************************/

bool Pixhawk_fsm::take_off(pixhawk_fsm::TakeOff::Request& request,
                           pixhawk_fsm::TakeOff::Response& response) {
    auto setpoint = std::make_shared<Setpoint_Data>();
    setpoint->offset.x = setpoint->offset.y = 0;
    setpoint->offset.z = request.height;
    Response attempt_response = attemptToCreateOperation(OperationIdentifier::TAKE_OFF, setpoint);
    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Pixhawk_fsm::explore(pixhawk_fsm::Explore::Request& request,
                          pixhawk_fsm::Explore::Response& response) {
    Response attempt_response = attemptToCreateOperation(OperationIdentifier::EXPLORE);

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Pixhawk_fsm::land(pixhawk_fsm::Land::Request& request, pixhawk_fsm::Land::Response& response) {
    Response attempt_response = attemptToCreateOperation(OperationIdentifier::LAND);

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

Pixhawk_fsm::Response Pixhawk_fsm::attemptToCreateOperation(
    const OperationIdentifier& target_operation_identifier,
    std::shared_ptr<Setpoint_Data> setPoint) {
    if (target_operation_identifier == OperationIdentifier::LAND) {
        Land();
    } else {
        std::cout << "call move!" << std::endl;
        Move(setPoint);
    }

    Response response;
    OperationIdentifier current_operation_identifier =
        getOperationIdentifierForOperation(current_operation_ptr);

    response.success = isValidOperation(current_operation_identifier, target_operation_identifier);

    if (!response.success) {
        response.message = "Cannot transition to " +
                           getStringFromOperationIdentifier(target_operation_identifier) +
                           " from " +
                           getStringFromOperationIdentifier(current_operation_identifier);
        std::cout << response.message << std::endl;
        return response;
    } else {
        ROS_INFO_STREAM(ros::this_node::getName().c_str()
                        << ": "
                        << "Transitioning to "
                        << getStringFromOperationIdentifier(target_operation_identifier).c_str());
    }

    return response;
}

std::shared_ptr<Operation> Pixhawk_fsm::performOperationTransition(
    std::shared_ptr<Operation> current_operation_ptr,
    std::shared_ptr<Operation> target_operation_ptr) {
    std::cout << "Now in performOperationTransition" << std::endl;
    OperationIdentifier current_operation_identifier =
        getOperationIdentifierForOperation(current_operation_ptr);
    OperationIdentifier target_operation_identifier =
        getOperationIdentifierForOperation(target_operation_ptr);
    std::cout << "current_operation: "
              << getStringFromOperationIdentifier(current_operation_identifier) << std::endl;
    std::cout << "target_operation: "
              << getStringFromOperationIdentifier(target_operation_identifier) << std::endl;

    ros::Rate rate(Pixhawk_fsm::getInstance().configuration.refresh_rate);
    MavrosInterface mavros_interface;

    // Loop until the Pixhawk mode is set.
    const std::string target_operation_pixhawk_mode =
        getPixhawkModeForOperationIdentifier(target_operation_ptr->identifier);
    while (ros::ok() && !mavros_interface.attemptToSetMode(target_operation_pixhawk_mode)) {
        std::cout << "Now in attemptToSetMode while loop" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    if (current_operation_ptr) {
        target_operation_ptr->current_pose = current_operation_ptr->getCurrentPose();
        target_operation_ptr->current_twist = current_operation_ptr->getCurrentTwist();
    }

    return target_operation_ptr;
}

/******************************************************************************************************
 *                                          State Machine *
 ******************************************************************************************************/

// Land drone external event
void Pixhawk_fsm::Land(void) {
    BEGIN_TRANSITION_MAP                     // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)      // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Takeoff
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Move
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Hold
        END_TRANSITION_MAP(NULL)
}

// move drone external event
void Pixhawk_fsm::Move(std::shared_ptr<Setpoint_Data> pData) {
    BEGIN_TRANSITION_MAP                     // - Current State -
    TRANSITION_MAP_ENTRY(ST_TAKEOFF)         // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Takeoff
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Move
        TRANSITION_MAP_ENTRY(ST_MOVE)        // ST_Hold
        END_TRANSITION_MAP(pData)
}

// state machine sits here when pixhawk is not running
void Pixhawk_fsm::ST_Idle(EventData* pData) {
    ROS_INFO_STREAM("[Keyboard_client]: Current state: IDLE");
}

void Pixhawk_fsm::ST_Land(EventData* pData) {}

void Pixhawk_fsm::ST_Takeoff(std::shared_ptr<Setpoint_Data> setpoint) {
    std::cout << "Now in ST_Takeoff" << std::endl;
    std::cout << setpoint->offset.z << std::endl;
    operation_execution_queue = {std::make_shared<TakeOffOperation>(setpoint->offset.z),
                                 std::make_shared<HoldOperation>()};

    got_new_operation = true;
    current_operation =
        getStringFromOperationIdentifier(operation_execution_queue.front()->identifier);
}

void Pixhawk_fsm::ST_Move(std::shared_ptr<Setpoint_Data> setpoint) {}

void Pixhawk_fsm::ST_Hold(EventData* pData) {
    std::cout << "Now in ST_Hold" << std::endl;
    operation_execution_queue = {std::make_shared<HoldOperation>()};
    got_new_operation = true;
    current_operation =
        getStringFromOperationIdentifier(operation_execution_queue.front()->identifier);
}

/******************************************************************************************************
 *                                          Helpers *
 ******************************************************************************************************/

OperationIdentifier Pixhawk_fsm::getOperationIdentifierForOperation(
    std::shared_ptr<Operation> operation_ptr) {
    if (!operation_ptr) {
        return OperationIdentifier::UNDEFINED;
    }

    return operation_ptr->identifier;
}

bool Pixhawk_fsm::isValidOperation(const OperationIdentifier& current_operation_identifier,
                                   const OperationIdentifier& target_operation_identifier) const {
    if (current_operation_identifier == target_operation_identifier) {
        return true;
    }

    return false;
}

/******************************************************************************************************
 *                                          Main Logic *
 ******************************************************************************************************/

void Pixhawk_fsm::run() {
    ros::Rate rate(configuration.refresh_rate);
    bool has_called_completion = false;

    while (ros::ok()) {
        got_new_operation = false;
        if (!operation_execution_queue.empty()) {
            std::cout << "operation_execution_queue's size: " << operation_execution_queue.size()
                      << std::endl;
            current_operation_ptr = performOperationTransition(current_operation_ptr,
                                                               operation_execution_queue.front());
            operation_execution_queue.pop_front();
            has_called_completion = false;
        }

        // If we are at the steady operation, we call the completion service
        if (operation_execution_queue.empty() && !has_called_completion) {
            std::cout << "operation_execution_queue is empty" << std::endl;
            pixhawk_fsm::OperationCompletion operation_completion;
            operation_completion.request.operation = current_operation;
            operation_completion_client.call(operation_completion);
            has_called_completion = true;
        }

        if (current_operation_ptr) {
            std::cout << "======" << std::endl;
            std::cout << "Current pixhawk mode: "
                      << getPixhawkModeForOperationIdentifier(current_operation_ptr->identifier)
                      << std::endl;
            std::cout << "Current Operation: " << current_operation << std::endl;
            std::cout << "======" << std::endl;

            getStatusPublisherPtr()->status.current_operation = current_operation;
            getStatusPublisherPtr()->status.pixhawk_mode =
                getPixhawkModeForOperationIdentifier(current_operation_ptr->identifier);
            current_operation_ptr->perform([&]() -> bool { return !got_new_operation; },
                                           operation_execution_queue.empty());
        }

        ros::spinOnce();
        rate.sleep();
    }
}