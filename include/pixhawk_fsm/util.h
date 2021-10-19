/**
 * @file util.h
 */

#ifndef UTIL_H
#define UTIL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/PositionTarget.h>
#include <termios.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include <cctype>
#include <vector>

/**
 * @brief Holds a bunch of convenience functions.
 */
class Util {
   public:
    /**
     * @param n Number you want to square
     *
     * @return The quare of @p n.
     */
    static double sq(double n) { return n * n; }

    /**
     * @brief If a number is negative, then we return the opposite of
     * the quared root of the absolut value of that number.
     *
     * @param nb Number you want to take the signed squared root
     *
     * @return The quare of @p n.
     */
    static double signed_sqrt(double nb) { return nb > 0 ? sqrt(nb) : -sqrt(-nb); }

    /**
     * @param n Number you want to square
     *
     * @return The quare of @p n.
     */
    static double moduloPi(double n) { return n - 2 * M_PI * floor((n + M_PI) / 2.0 / M_PI); }

    /**
     * @param current First point.
     * @param target Second point.
     *
     * @return Eucledian distance betweeen @p current and @p target.
     */
    static double distanceBetween(const geometry_msgs::Point& current,
                                  const geometry_msgs::Point& target) {
        double delta_x = target.x - current.x;
        double delta_y = target.y - current.y;
        double delta_z = target.z - current.z;

        return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    }

    /**
     * @brief Creates a path between @p first and @p last which consist of a series of points
     * between them specified by @p density.
     *
     * @param first The first point.
     * @param last The last point.
     * @param density The density of points between @p first and @p last.
     *
     * @return The new path with inserted points.
     */
    static std::vector<geometry_msgs::Point> createPath(const geometry_msgs::Point& first,
                                                        const geometry_msgs::Point& last,
                                                        const double& density) {
        double distance = distanceBetween(first, last);

        std::vector<geometry_msgs::Point> path;

        float delta_x = last.x - first.x;
        float delta_y = last.y - first.y;
        float delta_z = last.z - first.z;

        for (int i = 0; i < int(density * distance) + 1; i++) {
            geometry_msgs::Point temp;

            temp.x = first.x + i * delta_x / (density * distance);
            temp.y = first.y + i * delta_y / (density * distance);
            temp.z = first.z + i * delta_z / (density * distance);
            path.insert(path.end(), temp);
        }

        return path;
    }

    /**
     * @brief Sum positition, velocity and acceleration value from two PositionTarget.
     * Remark: the returned PositionTarget will have the same header as the first parameter
     *
     * @param a The first position target to sum and from which will be taken the header
     * @param b The second position target to sum
     * @return mavros_msgs::PositionTarget
     */
    static mavros_msgs::PositionTarget addPositionTarget(mavros_msgs::PositionTarget a,
                                                         mavros_msgs::PositionTarget b) {
        mavros_msgs::PositionTarget res;
        res.header = a.header;  // this is arbitrary. Did no find a perfect solution, but should not
                                // have any impact

        res.position.x = a.position.x + b.position.x;
        res.position.y = a.position.y + b.position.y;
        res.position.z = a.position.z + b.position.z;

        res.velocity.x = a.velocity.x + b.velocity.x;
        res.velocity.y = a.velocity.y + b.velocity.y;
        res.velocity.z = a.velocity.z + b.velocity.z;

        res.acceleration_or_force.x = a.acceleration_or_force.x + b.acceleration_or_force.x;
        res.acceleration_or_force.y = a.acceleration_or_force.y + b.acceleration_or_force.y;
        res.acceleration_or_force.z = a.acceleration_or_force.z + b.acceleration_or_force.z;

        return res;
    }

    /**
     * @brief Sum positition, velocity and acceleration value from two PoseStamped.
     * Remark: the returned PoseStamped will have the same header as the first parameter
     *
     * @param a The first position target to sum and from which will be taken the header
     * @param b The second position target to sum
     * @return geometry_msgs::PoseStamped
     */
    static geometry_msgs::PoseStamped addPoseStamped(geometry_msgs::PoseStamped a,
                                                     geometry_msgs::PoseStamped b) {
        geometry_msgs::PoseStamped res;
        res.header = a.header;  // this is arbitrary. Did no find a perfect solution, but should not
                                // have any impact

        res.pose.position.x = a.pose.position.x + b.pose.position.x;
        res.pose.position.y = a.pose.position.y + b.pose.position.y;
        res.pose.position.z = a.pose.position.z + b.pose.position.z;

        res.pose.orientation.x = a.pose.orientation.x + b.pose.orientation.x;
        res.pose.orientation.y = a.pose.orientation.y + b.pose.orientation.y;
        res.pose.orientation.z = a.pose.orientation.z + b.pose.orientation.z;
        res.pose.orientation.w = a.pose.orientation.w + a.pose.orientation.w;

        return res;
    }

    /**
     * @brief translate euler angle to Quaternion
     *
     * @param yaw The euler yaw (arround the z axis)
     * @param pitch The euler pitch (arround the y axis)
     * @param roll The euler roll (arround the x axis)
     * @return The equivalent quaternion
     */
    static geometry_msgs::Quaternion euler_to_quaternion(double yaw, double pitch, double roll) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::Quaternion quat;
        quat.w = q.getW();
        quat.x = q.getX();
        quat.y = q.getY();
        quat.z = q.getZ();

        return quat;
    }

    /**
     * @brief translate euler angle to Quaternion
     *
     * @param euler The euler angle we want to translate. y = pitch, x = roll, z = yaw
     * @return The equivalent quaternion
     */
    static geometry_msgs::Quaternion euler_to_quaternion(geometry_msgs::Vector3 euler) {
        return euler_to_quaternion(euler.z, euler.y, euler.x);
    }

    /**
     * @brief Transform an a quaternion orientation into euler angles.
     *
     * @param orientation The quaternion orientation.
     *
     * @return The equivalent euler angle.
     */
    static geometry_msgs::Vector3 quaternion_to_euler_angle(geometry_msgs::Quaternion orientation) {
        geometry_msgs::Vector3 eul;
        tf2::Quaternion q;
        q.setValue(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        m.getEulerYPR(eul.z, eul.y, eul.x);
        return eul;
    }

    /**
     * @brief Get keyboard input.
     *
     * @return Input character.
     */
    static char getch_noblocking() {
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
};

#endif
