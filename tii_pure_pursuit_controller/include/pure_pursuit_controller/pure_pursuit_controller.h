#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>  // Include for quaternion operations
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // Include for converting geometry messages

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void computeControlCommand();
    double calculateSteeringAngle(const geometry_msgs::PoseStamped& target_pose);
    geometry_msgs::PoseStamped getLookaheadPoint();

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    nav_msgs::Path path_;
    geometry_msgs::Pose current_pose_;

    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
};

#endif // PURE_PURSUIT_CONTROLLER_H
