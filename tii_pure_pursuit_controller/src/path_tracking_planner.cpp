#include "ros/ros.h"
#include "pure_pursuit_controller/path_tracking_planner.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>

PathTrackingPlanner::PathTrackingPlanner(ros::NodeHandle& nh)
    {
    odom_sub_ = nh.subscribe("/gem/base_footprint/odom", 10, &PathTrackingPlanner::odometryCallback, this);
    path_sub_ = nh.subscribe("/gem/path", 10, &PathTrackingPlanner::pathCallback, this);

    // std::string waypoints_file = "/root/catkin_ws/src/tii_pure_pursuit_controller/waypoint_examples/curved_waypoints.csv";
    // nh_.param<std::string>("waypoints_file", waypoints_file, waypoints_file);

    // global_path_ = loadWaypointsFromCSV(waypoints_file);
}

// void PathTrackingPlanner::run() {
//     ros::Rate loop_rate(10);  // 10 Hz
//     while (ros::ok()) {
//         if (!global_path_.empty()) {
//             std::vector<Waypoint> transformed_path = transformGlobalPathToVehicleFrame(global_path_, current_position_);
//             double steering_angle = pure_pursuit_.computeSteeringAngle(current_position_, transformed_path);
//             publishPath(global_path_, path_pub_);

//             geometry_msgs::Twist cmd_msg;
//             cmd_msg.linear.x = 1.0;  // Example speed, adjust as needed
//             cmd_msg.angular.z = steering_angle;

//             cmd_pub_.publish(cmd_msg);
//         }
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

std::vector<Waypoint> PathTrackingPlanner::loadWaypointsFromCSV(const std::string& file_path) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> waypoint_values;

        while (std::getline(ss, value, ',')) {
            waypoint_values.push_back(std::stod(value));
        }

        if (waypoint_values.size() >= 3) {
            waypoints.emplace_back(
                waypoint_values[0],  // X
                waypoint_values[1],  // Y
                waypoint_values[2]   // Yaw
            );
        }
    }

    file.close();
    return waypoints;
}

void PathTrackingPlanner::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_position.yaw = yaw;
    // Log the current position and yaw for debugging
    ROS_INFO("Current position: x = %f, y = %f, yaw = %f",
             current_position.x, current_position.y, current_position.yaw);

}

void PathTrackingPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // This function can be filled in to handle dynamic path updates
        global_path.clear();

    for (const auto& pose : msg->poses) {
        Waypoint wp(pose.pose.position.x, 
                    pose.pose.position.y, 
                    0.0); // Yaw will be calculated or provided separately
        global_path.push_back(wp);
    }

    ROS_INFO("Path received with %lu waypoints", global_path.size());

}

std::vector<Waypoint> PathTrackingPlanner::transformGlobalPathToVehicleFrame(const std::vector<Waypoint>& global_path, const Waypoint& current_position) {
    std::vector<Waypoint> transformed_path;
    for (const auto& global_wp : global_path) {
        // Translate the waypoint relative to the vehicle's position
        double dx = 0.0;
        double dy = -1.999994;

        double transformed_x = global_wp.x - dx;
        double transformed_y = global_wp.y - dy;

        // The yaw remains the difference in heading direction
        double transformed_yaw = global_wp.yaw - -0.000003;

        transformed_path.emplace_back(transformed_x, transformed_y, transformed_yaw);
    }
    
    return transformed_path;
}

void PathTrackingPlanner::publishPath(const std::vector<Waypoint>& path, ros::Publisher& path_pub) {
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = "base_footprint";

    for (const auto& wp : path) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_footprint";
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = std::cos(wp.yaw / 2);
        pose.pose.orientation.z = std::sin(wp.yaw / 2);
        ros_path.poses.push_back(pose);
        ROS_INFO("Waypoint : orientation.w = %f, orientation.z = %f",
                 pose.pose.orientation.w, pose.pose.orientation.z);
        
    }

    path_pub.publish(ros_path);
}

Waypoint PathTrackingPlanner::transformToVehicleFrame(const Waypoint& global_waypoint, const Waypoint& current_position) {
    // Translate the waypoint relative to the vehicle's position
    double dx = global_waypoint.x - current_position.x;
    double dy = global_waypoint.y - current_position.y;

    // Rotate the waypoint relative to the vehicle's orientation (yaw)
    double cos_yaw = std::cos(-current_position.yaw);
    double sin_yaw = std::sin(-current_position.yaw);

    double transformed_x = cos_yaw * dx - sin_yaw * dy;
    double transformed_y = sin_yaw * dx + cos_yaw * dy;

    // The yaw remains the difference in heading direction
    double transformed_yaw = global_waypoint.yaw - current_position.yaw;

    return Waypoint(transformed_x, transformed_y, transformed_yaw);
}