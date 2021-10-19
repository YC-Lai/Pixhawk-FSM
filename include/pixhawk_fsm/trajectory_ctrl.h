#ifndef _TRAJECTORY_CTRL_H
#define _TRAJECTORY_CTRL_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <memory>
#include <mutex>

#include "ctrl.h"

class Trajectory_ctrl : public Ctrl {
   public:
    Trajectory_ctrl(double height);

   private:
    ros::Subscriber marker_subscriber;
    mav_msgs::EigenTrajectoryPointDeque path;

    virtual void ST_Move(std::shared_ptr<EventData>) override;

    void markerCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg);
};

#endif  // _TRAJECTORY_CTRL_H
