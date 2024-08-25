#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void computeControlCommand();
    double calculateSteeringAngle(const geometry_msgs::PoseStamped& target_pose);
    geometry_msgs::PoseStamped getLookaheadPoint();  // Updated to take no arguments
    double adjustLookaheadDistance();  // Declaration of the new function
    double calculateCurvature(const geometry_msgs::PoseStamped& pose1,  // Declaration of the curvature calculation function
                              const geometry_msgs::PoseStamped& pose2, 
                              const geometry_msgs::PoseStamped& pose3);

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    nav_msgs::Path path_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_;

    double lookahead_distance_;
    double min_lookahead_distance_ = 0.5;  // Example default value
    double max_linear_velocity_;
    double max_angular_velocity_;
    double speed_factor_ = 0.1;  // Example default value
    double curvature_factor_ = 1.0;  // Example default value
};

#endif // PURE_PURSUIT_CONTROLLER_H
