#ifndef _KEYBOARD_CTRL_H
#define _KEYBOARD_CTRL_H
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pixhawk_fsm/KB_Travel.h>
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
     * @brief constant pixhawk parameter for the keyboard control.
     */
    double shift, shift_angle;

    void ST_Move(std::shared_ptr<EventData>) override;
};
#endif  // _KEYBOARD_CTRL_H