#include <ros/ros.h>

#include "trajectory_ctrl.h"
#include "util.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixhawk_fsm_client");
    ros::NodeHandle node_handle;

    int update_rate = 5;

    double takeoff_height = 1.5f;

    Trajectory_ctrl trajectory_ctrl(takeoff_height);
    auto data = std::make_shared<EventData>();;

    ros::Rate rate(update_rate);

    while (ros::ok()) {
        int c = Util::getch_noblocking();
        if (c == '1') {
            trajectory_ctrl.Move(data);
        } else if (c == '2') {
            trajectory_ctrl.Land();
        } else if (c == 't' || c == 'T') {
            trajectory_ctrl.Move(data);
        }

        ros::spinOnce();
        rate.sleep();
    }
}