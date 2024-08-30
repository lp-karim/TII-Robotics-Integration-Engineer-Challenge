#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "ros/ros.h"
#include <algorithm> 
#include <utility>
#include <numeric> 
#include <sstream>
#include <cmath>

PurePursuit::PurePursuit(double lookahead_distance, double wheelbase, ros::Publisher& marker_pub)
    : lookahead_distance_(lookahead_distance), wheelbase_(wheelbase), marker_pub_(marker_pub),
    current_waypoint_index_(0){}

std::pair<double, int> PurePursuit::computeSteeringAngle(const Waypoint& current_position, const std::vector<Waypoint>& path) {
    int target_index = findTargetWaypointIndex(current_position, path);
    Waypoint target_wp = path[target_index];

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
    double alpha = target_wp.yaw - current_position.yaw;
    ROS_INFO("Alpha (angle to target): %f", alpha);
    // Dynamic LookAhead Distance
    lookahead_distance_ = computeDistance(current_position, target_wp);
    ROS_INFO("True LookAhead distance: %f", lookahead_distance_);
    // Calculate the steering angle delta
    double k       = 0.285;
    double delta = std::atan2(2.0 * k * wheelbase_ * std::sin(alpha) / lookahead_distance_, 1.0);
    delta *= 2.0;  // 0.7 // Scaling down to reduce aggressiveness
    delta = std::max(-0.61, std::min(delta, 0.61));
    delta = std::round(delta * 1000.0) / 1000.0;

    double ct_error = std::round(std::sin(alpha) * wheelbase_ * 1000.0) / 1000.0;
    ROS_INFO("Computed Steering Angle: %f", delta);
    
    return std::make_pair(delta, target_index);
}

int PurePursuit::findTargetWaypointIndex(const Waypoint& current_position, const std::vector<Waypoint>& path) {
    double distance_to_current_wp = computeDistance(current_position, path[current_waypoint_index_]);

    ROS_INFO("Distance to current waypoint: %f", distance_to_current_wp);
    ROS_INFO("Current waypoint index: %d", current_waypoint_index_);

    // Finding the distance of each waypoint from the current position
    std::vector<double> dist_arr(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        double dx = path[i].x - current_position.x;
        double dy = path[i].y - current_position.y;
        dist_arr[i] = std::hypot(dx, dy);  // Equivalent to your dist() function
    }

    // Finding those points which are less than the lookahead distance
    std::vector<int> goal_arr;
    for (size_t i = 0; i < dist_arr.size(); ++i) {
        if (dist_arr[i] < lookahead_distance_ + 0.3 && dist_arr[i] > lookahead_distance_ - 0.3) {
            goal_arr.push_back(static_cast<int>(i));
        }
    }
    // Debugging: Log the contents of goal_arr 
    // std::stringstream ss;
    // ss << "Contents of goal_arr: ";
    // for (int idx : goal_arr) {
    //     ss << idx << " ";
    // }
    // ROS_INFO("%s", ss.str().c_str());
    // Finding the goal point which is the last in the set of points less than the lookahead distance
    for (int idx : goal_arr) {
        std::vector<double> v1 = {path[idx].x - current_position.x, path[idx].y - current_position.y};
        std::vector<double> v2 = {std::cos(current_position.yaw), std::sin(current_position.yaw)};
        // ROS_INFO("vector: %f,%f", v1[0],v1[1]);
        double temp_angle = find_angle(v1, v2);  
        // ROS_INFO("angle: %f",temp_angle);
        if (std::abs(temp_angle) < M_PI / 2) {
            current_waypoint_index_ = idx;
            break;
        }
    }
    lookahead_distance_ = dist_arr[current_waypoint_index_];
    ROS_INFO("Switching to next waypoint: %d", current_waypoint_index_);
        
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


// Function to find the angle between two vectors
double PurePursuit::find_angle(const std::vector<double>& v1, const std::vector<double>& v2) {

    // Calculate the dot product
    double cosang = std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0);

    // Calculate the cross product magnitude (for 2D vectors)
    double sinang = std::abs(v1[0] * v2[1] - v1[1] * v2[0]);

    // Return the angle in the range [-pi, pi]
    return std::atan2(sinang, cosang);
}

// Function to calculate the dot product of two vectors
double PurePursuit::dot_product(const std::vector<double>& v1, const std::vector<double>& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1];
}

// Function to calculate the magnitude of a vector
double PurePursuit::magnitude(const std::vector<double>& v) {
    return std::sqrt(v[0] * v[1] + v[1] * v[1]);
}