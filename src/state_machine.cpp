#include "state_machine.h"

#include <assert.h>
#include <iostream>

StateMachine::StateMachine(unsigned char maxStates)
    : _maxStates(maxStates), currentState(0), _eventGenerated(false), _pEventData(NULL) {}

// generates an external event. called once per external event
// to start the state machine executing
void StateMachine::ExternalEvent(unsigned char newState, std::shared_ptr<EventData> pData) {
    // if we are supposed to ignore this event
    if (newState == EVENT_IGNORED) {
        std::cout << "Cannot transition" << std::endl;
        isValidOperation = false;
        // just delete the event data, if any
        // if (pData) delete pData;
    } else {
        // TODO - capture software lock here for thread-safety if necessary

        // generate the event and execute the state engine
        isValidOperation = true;
        InternalEvent(newState, pData);
        StateEngine();

        // TODO - release software lock here
    }
}

// generates an internal event. called from within a state
// function to transition to a new state
void StateMachine::InternalEvent(unsigned char newState, std::shared_ptr<EventData> pData) {
    // if (pData == NULL) pData = new EventData();

    _pEventData = pData;
    _eventGenerated = true;
    currentState = newState;
}

// the state engine executes the state machine states
void StateMachine::StateEngine(void) {
    EventData* pDataTemp = NULL;

    // while events are being generated keep executing states
    while (_eventGenerated) {
        // pDataTemp = _pEventData;  // copy of event data pointer
        // _pEventData = NULL;       // event data used up, reset ptr
        _eventGenerated = false;  // event used up, reset flag

        assert(currentState < _maxStates);

        // get state map
        const StateStruct* pStateMap = GetStateMap();

        // execute the state passing in event data, if any
        (this->*pStateMap[currentState].pStateFunc)(_pEventData);

        // if event data was used, then delete it
        if (pDataTemp) {
            delete pDataTemp;
            pDataTemp = NULL;
        }
    }
}