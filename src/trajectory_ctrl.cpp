#include "trajectory_ctrl.h"

Trajectory_ctrl::Trajectory_ctrl(double height) : Ctrl(height) {
    marker_subscriber = node_handle.subscribe("/firefly/command/trajectory", 10,
                                              &Trajectory_ctrl::markerCallback, this);
}

void Trajectory_ctrl::markerCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg) {
    std::lock_guard<std::mutex> pose_guard(*(state_mutex_));
    eigenTrajectoryPointDequeFromMsg(msg, &path);
}

void Trajectory_ctrl::ST_Move(std::shared_ptr<EventData>) {
    ROS_INFO_STREAM("[Client]: Current state: MOVE");
    std::vector<geometry_msgs::Point> publish_path;
    while (!path.empty()) {
        mav_msgs::EigenTrajectoryPoint waypoint = path.front();
        path.pop_front();

        geometry_msgs::Point publish_point;
        publish_point.x = waypoint.position_W(0);
        publish_point.y = waypoint.position_W(1);
        publish_point.z = waypoint.position_W(2);

        publish_path.emplace_back(publish_point);
    }

    geometry_msgs::Point target_point = *(publish_path.end());

    pixhawk_fsm::Explore explore_service_handle;
    explore_service_handle.request.path = publish_path;
    explore_service_handle.request.point_of_interest = target_point;

    if (explore.call(explore_service_handle)) {
        if (!explore_service_handle.response.success) {
            ROS_FATAL_STREAM(explore_service_handle.response.message);
        } else {
            is_executing_operation = true;
        }
    } else {
        ROS_FATAL("Failed to call explore service.");
    }
}