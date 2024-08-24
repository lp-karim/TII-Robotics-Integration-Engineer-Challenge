#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh);

    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    nav_msgs::Path path_;
    geometry_msgs::Pose current_pose_;

    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    void computeControlCommand();
    geometry_msgs::PoseStamped getLookaheadPoint();
};

#endif  // PURE_PURSUIT_CONTROLLER_H
