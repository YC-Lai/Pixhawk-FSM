#ifndef _KEYBOARD_CTRL_H
#define _KEYBOARD_CTRL_H
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <pixhawk_fsm/Land.h>
#include <pixhawk_fsm/OperationCompletion.h>
#include <pixhawk_fsm/TakeOff.h>
#include <pixhawk_fsm/Travel.h>
#include <ros/ros.h>

#include <memory>
#include <mutex>

#include "ctrl.h"

#define PI (3.1415926)

enum class MANEUVER {
    IDLE,
    TAKEOFF,
    LAND,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN,
    TURNLEFT,
    TURNRIGHT
};

// structure to hold event data passed into state machine
struct Keyboard_data : public EventData {
    MANEUVER input;
};

// the Keyboard_ctrl state machine class
class Keyboard_ctrl : public Ctrl {
   public:
    Keyboard_ctrl(double height, double speed, int update_rate);

   private:
    /**
     * @brief constant speed ardupilot parameter for the keyboard control.
     */
    double move_speed;
    int update_rate;
    double path_density;

    void ST_Move(std::shared_ptr<EventData>) override;
};
#endif  // _KEYBOARD_CTRL_H