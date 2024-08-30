#ifndef PATH_TRACKING_PLANNER_H
#define PATH_TRACKING_PLANNER_H

#include "ros/ros.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <tf/transform_listener.h>

class PathTrackingPlanner {
public:
    PathTrackingPlanner(ros::NodeHandle& nh);
    std::vector<Waypoint> loadWaypointsFromCSV(const std::string& file_path);
    std::vector<Waypoint> transformGlobalPathToVehicleFrame(const std::vector<Waypoint>& global_path, const Waypoint& current_position);
    void publishPath(const std::vector<Waypoint>& path, ros::Publisher& path_pub);
    Waypoint getCurrentPosition() const {
        return current_position;
    }
    // void run();

private:
    ros::Publisher path_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    std::vector<Waypoint> global_path;
    Waypoint current_position;
    tf::TransformListener tf_listener_;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);


};

#endif // PATH_TRACKING_PLANNER_H