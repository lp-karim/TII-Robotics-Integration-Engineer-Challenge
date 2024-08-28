#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "pure_pursuit_controller/frame_id_remapper.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>


// Global variables to store the path and the current odometry
std::vector<Waypoint> global_path;
Waypoint current_position(0.0, 0.0, 0.0);

std::vector<Waypoint> loadWaypointsFromCSV(const std::string& filepath) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filepath.c_str());
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

void publishPath(const std::vector<Waypoint>& waypoints, ros::Publisher& path_pub) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "base_footprint"; 
    
    for (const auto& wp : waypoints) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base_footprint";
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = std::cos(wp.yaw / 2);
        pose.pose.orientation.z = std::sin(wp.yaw / 2); 
        
        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Extract the position and orientation (yaw) from the odometry message
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                            msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_position.yaw = std::atan2(siny_cosp, cosy_cosp);

    // Log the current position and yaw for debugging
    ROS_INFO("Current position: x = %f, y = %f, yaw = %f",
             current_position.x, current_position.y, current_position.yaw);

}

void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // Log the frame ID
    // ROS_INFO("Path frame ID: %s", msg->header.frame_id.c_str());
    global_path.clear();

    for (const auto& pose : msg->poses) {
        Waypoint wp(pose.pose.position.x, 
                    pose.pose.position.y, 
                    0.0); // Yaw will be calculated or provided separately
        global_path.push_back(wp);
    }

    ROS_INFO("Path received with %lu waypoints", global_path.size());

}

void printWaypoints(const std::vector<Waypoint>& waypoints) {
    ROS_INFO("Loaded %lu waypoints:", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
        ROS_INFO("Waypoint %lu: x = %f, y = %f, yaw = %f",
                 i, waypoints[i].x, waypoints[i].y, waypoints[i].yaw);
    }
}


// Function to transform waypoints relative to the vehicle's position
std::vector<Waypoint> transformGlobalPathToVehicleFrame(const std::vector<Waypoint>& global_path, const Waypoint& current_position) {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller_node");
    ros::NodeHandle nh;

    // Use the simplified path
    std::vector<Waypoint> simplified_path = createSimplifiedPath();

    // Parameters for input and output topics, and new frame_id
    std::string input_topic = "/gem/velodyne_points";
    std::string output_topic = "/gem/velodyne_points";
    std::string new_frame_id = "velodyne_base_link";

    // Create the FrameIdRemapper object
    FrameIdRemapper remapper(nh, input_topic, output_topic, new_frame_id);


    // Initialize the marker publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Parameters
    std::string waypoints_file = "/root/catkin_ws/src/POLARIS_GEM_e2-main/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv";
    nh.param<std::string>("waypoints_file", waypoints_file);
    double lookahead_distance = 6.0;  // Can be parameterized
    double wheelbase = 1.5;  // Example value, adjust based on your robot

    ROS_INFO("Attempting to open file: %s", waypoints_file.c_str());

    // Load waypoints from CSV
    global_path = loadWaypointsFromCSV(waypoints_file);

    // Verify and print the loaded waypoints
    // printWaypoints(global_path);

    // ROS publishers
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/gem/path", 10);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);

    // ROS subscribers
    ros::Subscriber odom_sub = nh.subscribe("/gem/base_footprint/odom", 10, odometryCallback);
    ros::Subscriber path_sub = nh.subscribe("/gem/path", 10, pathCallback);

    // Pure Pursuit controller instance
    PurePursuit pure_pursuit(lookahead_distance, wheelbase, marker_pub);

    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        if (!global_path.empty()) {
    
            // Transform the global path into the vehicle's coordinate frame
            std::vector<Waypoint> transformed_path = transformGlobalPathToVehicleFrame(global_path, current_position);

            double steering_angle = pure_pursuit.computeSteeringAngle(current_position, transformed_path);

            publishPath(global_path, path_pub);
            
            // Create and publish the Twist message
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = 1.0;  // Example speed, adjust as needed
            cmd_msg.angular.z = steering_angle;

            cmd_pub.publish(cmd_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}