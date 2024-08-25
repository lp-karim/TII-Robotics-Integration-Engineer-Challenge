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

    // Dynamic adjustment of lookahead distance
    double dynamic_lookahead_distance = adjustLookaheadDistance();

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

    // Log important data
    ROS_INFO("Steering Angle: %f, Linear Velocity: %f, Angular Velocity: %f",
             steering_angle, linear_velocity, cmd_vel.angular.z);
    ROS_INFO("Dynamic Lookahead Distance: %f", dynamic_lookahead_distance);
}

// Function to adjust the lookahead distance based on speed or curvature
double PurePursuitController::adjustLookaheadDistance() {
    double current_speed = std::hypot(current_velocity_.linear.x, current_velocity_.linear.y);

    // Select three consecutive points from the path (ensure they are available)
    if (path_.poses.size() < 3) {
        ROS_WARN("Not enough points to calculate curvature");
        return lookahead_distance_;
    }

    // Calculate curvature using three consecutive points
    double curvature = calculateCurvature(path_.poses[0], path_.poses[1], path_.poses[2]);

    // Example: Reduce lookahead distance for higher curvature (sharper turns)
    double dynamic_lookahead_distance = std::max(min_lookahead_distance_, 
                                                 lookahead_distance_ / (1.0 + curvature_factor_ * curvature));

    // Optionally, adjust based on speed as well
    dynamic_lookahead_distance = std::max(min_lookahead_distance_, 
                                          dynamic_lookahead_distance - speed_factor_ * current_speed);

    return dynamic_lookahead_distance;
}
double PurePursuitController::calculateCurvature(const geometry_msgs::PoseStamped& pose1, 
                                                 const geometry_msgs::PoseStamped& pose2, 
                                                 const geometry_msgs::PoseStamped& pose3) {
    // Extract the coordinates of the three points
    double x1 = pose1.pose.position.x, y1 = pose1.pose.position.y;
    double x2 = pose2.pose.position.x, y2 = pose2.pose.position.y;
    double x3 = pose3.pose.position.x, y3 = pose3.pose.position.y;

    // Calculate the area of the triangle formed by the points
    double area = std::abs(0.5 * (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2)));

    // Calculate the side lengths of the triangle
    double d12 = std::hypot(x2 - x1, y2 - y1);
    double d23 = std::hypot(x3 - x2, y3 - y2);
    double d31 = std::hypot(x1 - x3, y1 - y3);

    // Calculate the curvature
    double curvature = 4 * area / (d12 * d23 * d31);
    return curvature;
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
