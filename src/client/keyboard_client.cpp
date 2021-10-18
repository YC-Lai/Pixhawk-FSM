#include <ros/ros.h>
#include <termios.h>

#include <cctype>
#include <memory>

#include "keyboard_ctrl.h"

char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
    return (buf);
}

char getch_noblocking() {
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0) ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0) ROS_ERROR("tcsetattr ICANON");

    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0)
        ;
    // ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len);

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0) ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_client");
    ros::NodeHandle node_handle;

    int update_rate = 5;
    double speed = 5;
    Keyboard_ctrl Keyboard_ctrl(speed, update_rate);
    auto data = std::make_shared<Keyboard_data>();

    ros::Rate rate(update_rate);

    while (ros::ok()) {
        int c = getch_noblocking();
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