#include "pixhawk_fsm.h"

#include <assert.h>

#include <iostream>

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

// Land drone external event
void Pixhawk_fsm::Land(void) {
    BEGIN_TRANSITION_MAP                     // - Current State -
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)  // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Takeoff
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Move
        TRANSITION_MAP_ENTRY(ST_LAND)        // ST_Hold
        END_TRANSITION_MAP(NULL)
}

// move drone external event
void Pixhawk_fsm::Move(void) {
    BEGIN_TRANSITION_MAP                     // - Current State -
        TRANSITION_MAP_ENTRY(ST_TAKEOFF)     // ST_Idle
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Land
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Takeoff
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)  // ST_Move
        TRANSITION_MAP_ENTRY(ST_MOVE)        // ST_Hold
        END_TRANSITION_MAP(NULL)
}

// state machine sits here when pixhawk is not running
void Pixhawk_fsm::ST_Idle(EventData* pData) {
    ROS_INFO_STREAM("[Keyboard_client]: Current state: IDLE");
}

void Pixhawk_fsm::ST_Land(EventData* pData) {}

void Pixhawk_fsm::ST_Takeoff(std::shared_ptr<Pixhawk_fsmData> setpoint) {}

void Pixhawk_fsm::ST_Move(std::shared_ptr<Pixhawk_fsmData> setpoint) {}

void Pixhawk_fsm::ST_Hold(std::shared_ptr<Pixhawk_fsmData> setpoint) {}
