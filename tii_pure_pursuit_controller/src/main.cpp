#include <ros/ros.h>
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Waypoint {
    double x;
    double y;
    double z;
    double qx;
    double qy;
};

std::vector<Waypoint> loadWaypoints(const std::string& file_path) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Waypoint wp;
        ss >> wp.x >> wp.y >> wp.z >> wp.qx >> wp.qy;
        waypoints.push_back(wp);
        // ROS_INFO("Loaded waypoint: x=%f, y=%f, z=%f, qx=%f, qy=%f", 
        //          wp.x, wp.y, wp.z, wp.qx, wp.qy);
    }
    ROS_INFO("Total waypoints loaded: %lu", waypoints.size());
    return waypoints;
}

nav_msgs::Path generatePath(const std::vector<Waypoint>& waypoints) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    for (const auto& wp : waypoints) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = wp.z;

        // Assuming qx, qy are part of the quaternion, and setting default values for qz, qw
        pose.pose.orientation.x = wp.qx;
        pose.pose.orientation.y = wp.qy;
        pose.pose.orientation.z = 0.0; // You may need to adjust this
        pose.pose.orientation.w = 1.0; // Assuming no rotation for simplicity

        ROS_INFO("Path Pose: x=%f, y=%f, z=%f, qx=%f, qy=%f", 
                 wp.x, wp.y, wp.z, wp.qx, wp.qy);
        path.poses.push_back(pose);
    }

    return path;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle nh;

    // Load waypoints from the CSV file
    std::vector<Waypoint> waypoints = loadWaypoints("/root/catkin_ws/src/POLARIS_GEM_e2-main/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv");

    // Generate a ROS path message from the waypoints
    nav_msgs::Path path = generatePath(waypoints);

    // Initialize the Pure Pursuit Controller
    PurePursuitController controller(nh);

    // Publish the path once
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);

    // Give ROS some time to initialize the publisher and subscribers
    ros::Duration(1.0).sleep();

    // Update the timestamp and publish the path
    path.header.stamp = ros::Time::now();
    for (auto& pose : path.poses) {
        pose.header.stamp = ros::Time::now();
    }
    path_pub.publish(path);
    ROS_INFO("Path published with %lu poses", path.poses.size());

    // Let the ROS node spin to handle callbacks
    ros::spin();

    return 0;
}