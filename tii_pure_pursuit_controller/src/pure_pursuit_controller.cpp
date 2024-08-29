#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "ros/ros.h"
#include <utility>

PurePursuit::PurePursuit(double lookahead_distance, double wheelbase, ros::Publisher& marker_pub)
    : lookahead_distance_(lookahead_distance), wheelbase_(wheelbase), marker_pub_(marker_pub),
    current_waypoint_index_(0), waypoint_reach_threshold_(0.05) {}

std::pair<double, int> PurePursuit::computeSteeringAngle(const Waypoint& current_position, const std::vector<Waypoint>& path) {
    int target_index = findTargetWaypointIndex(current_position, path);
    Waypoint target_wp = path[target_index];

    // Transform the global waypoint to the vehicle's coordinate frame
    // Waypoint target_wp = transformToVehicleFrame(global_target_wp,current_position);

    // Detailed logging
    ROS_INFO("Current Position: x = %f, y = %f, yaw = %f", current_position.x, current_position.y, current_position.yaw);
    ROS_INFO("Target Waypoint Index: %d", target_index);
    ROS_INFO("Target Waypoint: x = %f, y = %f, yaw = %f", target_wp.x, target_wp.y, target_wp.yaw);
    ROS_INFO("Distance to Target: %f", computeDistance(current_position, target_wp));


    // Publish a marker at the target waypoint
    publishTargetWaypointMarker(target_wp, target_index);

    // Calculate alpha, the angle between the vehicle's heading and the line connecting it to the target waypoint
    double dx = target_wp.x - current_position.x;
    double dy = target_wp.y - current_position.y;
    double alpha = std::atan2(dy, dx) - current_position.yaw;
    ROS_INFO("Alpha (angle to target): %f", alpha);

    // Calculate the steering angle delta
    double delta = std::atan2(2.0 * wheelbase_ * std::sin(alpha) / lookahead_distance_, 1.0);
    delta *= 1.0;  // 0.7 // Scaling down to reduce aggressiveness
    ROS_INFO("Computed Steering Angle: %f", delta);
    
    return std::make_pair(delta, target_index);
}

int PurePursuit::findTargetWaypointIndex(const Waypoint& current_position, const std::vector<Waypoint>& path) {
    double distance_to_current_wp = computeDistance(current_position, path[current_waypoint_index_]);

    ROS_INFO("Distance to current waypoint: %f", distance_to_current_wp);
    ROS_INFO("Current waypoint index: %d", current_waypoint_index_);

    if (distance_to_current_wp < waypoint_reach_threshold_) {
        // Move to the next waypoint if the current one is reached
        if (current_waypoint_index_ < path.size() - 1) {
            current_waypoint_index_++;
            ROS_INFO("Switching to next waypoint: %d", current_waypoint_index_);
        }
    }

    // Continue tracking the current waypoint until it's reached
    return current_waypoint_index_;
    }


double PurePursuit::computeDistance(const Waypoint& p1, const Waypoint& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

void PurePursuit::publishTargetWaypointMarker(const Waypoint& target_wp, int id) {
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "base_footprint";  // Adjust this to match your fixed frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_waypoint";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = target_wp.x;
    marker.pose.position.y = target_wp.y;
    marker.pose.position.z = 0.0;

    marker.scale.x = 0.5;  // Set the size of the sphere
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.a = 1.0;  // Set the transparency
    marker.color.r = 1.0;  // Set the color to red
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_pub_.publish(marker);
}

Waypoint PurePursuit::transformToVehicleFrame(const Waypoint& global_waypoint, const Waypoint& current_position) {
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