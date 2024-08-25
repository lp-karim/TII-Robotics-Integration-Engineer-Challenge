#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "pure_pursuit_controller/pure_pursuit_controller.h"

// Constructor: Initializes the node handle, subscribers, and publishers
PurePursuitController::PurePursuitController(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize the path and odometry subscribers
    path_sub_ = nh_.subscribe("path", 10, &PurePursuitController::pathCallback, this);
    odom_sub_ = nh_.subscribe("base_footprint/odom", 10, &PurePursuitController::odometryCallback, this);

    // Initialize the velocity command publisher
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

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
    computeControlCommand();
}

// Compute the control command to follow the path
void PurePursuitController::computeControlCommand() {
    if (path_.poses.empty()) {
        ROS_WARN("No path available.");
        return;
    }

    // Find the closest point on the path to the current position
    geometry_msgs::PoseStamped target_pose = getLookaheadPoint();

    // Calculate the steering angle based on the lookahead point
    double steering_angle = calculateSteeringAngle(target_pose);

    // Scale the linear velocity based on the steering angle
    double linear_velocity = max_linear_velocity_ * (1.0 - std::abs(steering_angle) / max_angular_velocity_);

    // Publish the velocity command
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = steering_angle;
    cmd_pub_.publish(cmd_vel);
}

// Function to calculate the steering angle to the target lookahead point
double PurePursuitController::calculateSteeringAngle(const geometry_msgs::PoseStamped& target_pose) {
    double dx = target_pose.pose.position.x - current_pose_.position.x;
    double dy = target_pose.pose.position.y - current_pose_.position.y;

    // Manually calculate yaw from the quaternion
    double qx = current_pose_.orientation.x;
    double qy = current_pose_.orientation.y;
    double qz = current_pose_.orientation.z;
    double qw = current_pose_.orientation.w;

    double yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    double local_x = cos(yaw) * dx + sin(yaw) * dy;
    double local_y = -sin(yaw) * dx + cos(yaw) * dy;

    return 2.0 * local_y / (lookahead_distance_ * lookahead_distance_);
}


// Function to find the lookahead point on the path
geometry_msgs::PoseStamped PurePursuitController::getLookaheadPoint() {
    geometry_msgs::PoseStamped lookahead_point;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& pose : path_.poses) {
        double distance = hypot(pose.pose.position.x - current_pose_.position.x,
                                pose.pose.position.y - current_pose_.position.y);
        if (distance > lookahead_distance_ && distance < min_distance) {
            min_distance = distance;
            lookahead_point = pose;
        }
    }
    return lookahead_point;
}
