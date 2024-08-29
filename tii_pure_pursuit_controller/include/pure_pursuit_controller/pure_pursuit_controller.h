#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <limits>
#include <tf/transform_listener.h>

struct Waypoint {
    double x;
    double y;
    double yaw;
    
    // Default constructor
    Waypoint() : x(0.0), y(0.0), yaw(0.0) {}

    Waypoint(double x_, double y_, double yaw_)
        : x(x_), y(y_), yaw(yaw_) {}
};

class PurePursuit {
public:
    PurePursuit(double lookahead_distance, double wheelbase, ros::Publisher& marker_pub);
    std::pair<double, int> computeSteeringAngle(const Waypoint& current_position, const std::vector<Waypoint>& path);

private:
    double lookahead_distance_ =6.0;
    double wheelbase_=1.5;
    double waypoint_reach_threshold_=0.05;
    int current_waypoint_index_;
    ros::Publisher& marker_pub_; 
    tf::TransformListener tf_listener_;
    int findTargetWaypointIndex(const Waypoint& current_position, const std::vector<Waypoint>& path);
    double computeDistance(const Waypoint& p1, const Waypoint& p2);
    void publishTargetWaypointMarker(const Waypoint& target_wp, int id);
    Waypoint transformToVehicleFrame(const Waypoint& global_waypoint, const Waypoint& current_position);
};

#endif // PURE_PURSUIT_CONTROLLER_H
