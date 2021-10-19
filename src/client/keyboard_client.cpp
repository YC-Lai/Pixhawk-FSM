#include <ros/ros.h>

#include <memory>

#include "keyboard_ctrl.h"
#include "util.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixhawk_fsm_client");
    ros::NodeHandle node_handle;

    double takeoff_height = 1.5f;
    double speed = 10;
    int update_rate = 5;

    Keyboard_ctrl Keyboard_ctrl(takeoff_height, speed, update_rate);
    auto data = std::make_shared<Keyboard_data>();

    ros::Rate rate(update_rate);

    while (ros::ok()) {
        int c = Util::getch_noblocking();
        if (c == '1') {
            data->input = MANEUVER::TAKEOFF;
            Keyboard_ctrl.Move(data);
        } else if (c == '2') {
            Keyboard_ctrl.Land();
        } else if (c == 'w' || c == 'W') {
            data->input = MANEUVER::FORWARD;
            Keyboard_ctrl.Move(data);
        } else if (c == 's' || c == 'S') {
            data->input = MANEUVER::BACKWARD;
            Keyboard_ctrl.Move(data);
        } else if (c == 'a' || c == 'A') {
            data->input = MANEUVER::LEFT;
            Keyboard_ctrl.Move(data);
        } else if (c == 'd' || c == 'D') {
            data->input = MANEUVER::RIGHT;
            Keyboard_ctrl.Move(data);
        }

        ros::spinOnce();
        rate.sleep();
    }
}