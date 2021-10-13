#ifndef PIXHAWK_FSM_H
#define PIXHAWK_FSM_H
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "state_machine.h"

// structure to hold event data passed into state machine
struct Pixhawk_fsmData : public EventData {
    geometry_msgs::Point offset;
};

class Pixhawk_fsm : public StateMachine {
   public:
    Pixhawk_fsm();

    // external events taken by this state machine
    void Move();

   private:
    // ros module
    ros::NodeHandle node_handle;

    // state machine state functions
    void ST_Idle(EventData*);
    void ST_Land(EventData*);
    void ST_Takeoff(std::shared_ptr<Pixhawk_fsmData>);
    void ST_Move(std::shared_ptr<Pixhawk_fsmData>);
    void ST_Hold(std::shared_ptr<Pixhawk_fsmData>);

    // state map to define state function order
    BEGIN_STATE_MAP
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Idle)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Land)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Takeoff)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Move)
    STATE_MAP_ENTRY(&Pixhawk_fsm::ST_Hold)
    END_STATE_MAP

    // state enumeration order must match the order of state
    // method entries in the state map
    enum E_States { ST_IDLE = 0, ST_LAND, ST_TAKEOFF, ST_MOVE, ST_Hold, ST_MAX_STATES };
};

#endif  // PIXHAWK_FSM_H