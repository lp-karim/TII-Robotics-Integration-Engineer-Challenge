#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include "pure_pursuit_controller/pure_pursuit_controller.h"

// Constructor: Initializes the node handle, subscribers, and publishers
PurePursuitController::PurePursuitController(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize the path and odometry subscribers
    path_sub_ = nh_.subscribe("path", 1000, &PurePursuitController::pathCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1000, &PurePursuitController::odometryCallback, this);

    // Initialize the velocity command publisher
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // Load parameters from the parameter server or set defaults
    nh_.param("lookahead_distance", lookahead_distance_, 1.0);
    nh_.param("max_linear_velocity", max_linear_velocity_, 2.0);
    nh_.param("max_angular_velocity", max_angular_velocity_, 1.0);
}

// Callback function for receiving the path data
void PurePursuitController::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    path_ = *msg;
}

// Callback function for receiving the odometry data
void PurePursuitController::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose_ = msg->pose.pose;
    computeControlCommand(); // Compute the control command based on the current odometry
}

// Compute the control command to follow the path
void PurePursuitController::computeControlCommand() {
    if (path_.poses.empty()) {
        ROS_WARN("No path available.");
        return;
    }

    // Find the lookahead point on the path
    geometry_msgs::PoseStamped lookahead_point = getLookaheadPoint();

    // Transform the lookahead point to the vehicle's coordinate frame
    tf2::Transform tf_map_to_base;
    tf2::convert(current_pose_, tf_map_to_base);
    tf2::Transform tf_map_to_lookahead;
    tf2::convert(lookahead_point.pose, tf_map_to_lookahead);
    tf2::Transform tf_base_to_lookahead = tf_map_to_base.inverse() * tf_map_to_lookahead;

    // Get the coordinates of the lookahead point in the vehicle's frame
    double lx = tf_base_to_lookahead.getOrigin().x();
    double ly = tf_base_to_lookahead.getOrigin().y();

    // Calculate the curvature (kappa) needed to reach the lookahead point
    double curvature = 2 * ly / (lookahead_distance_ * lookahead_distance_);

    // Compute the control commands
    geometry_msgs::Twist cmd;
    cmd.linear.x = max_linear_velocity_;
    cmd.angular.z = curvature * max_linear_velocity_;

    // Limit the angular velocity to avoid sharp turns
    if (cmd.angular.z > max_angular_velocity_) {
        cmd.angular.z = max_angular_velocity_;
    } else if (cmd.angular.z < -max_angular_velocity_) {
        cmd.angular.z = -max_angular_velocity_;
    }

    // Publish the computed velocity command
    cmd_pub_.publish(cmd);
}

// Find the lookahead point on the path that the vehicle should aim for
geometry_msgs::PoseStamped PurePursuitController::getLookaheadPoint() {
    geometry_msgs::PoseStamped lookahead_point;
    double min_distance = std::numeric_limits<double>::max();

    // Iterate through the path to find the closest point that is beyond the lookahead distance
    for (const auto& pose : path_.poses) {
        double distance = std::hypot(
            pose.pose.position.x - current_pose_.position.x,
            pose.pose.position.y - current_pose_.position.y
        );

        if (distance > lookahead_distance_ && distance < min_distance) {
            min_distance = distance;
            lookahead_point = pose;
        }
    }

    return lookahead_point;
}
