#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "ros/ros.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "pure_pursuit_controller/path_tracking_planner.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <limits>
#include <utility>
#include <tf/transform_listener.h>

enum class VehicleState {
    Idle,
    PathFollowing,
    Completed
};

class StateMachine {
public:
    StateMachine(PurePursuit& controller, PathTrackingPlanner& planner, std::string& waypoints_file);

    void run(std::vector<Waypoint>& global_path_);  // Main loop for the state machine

private:
    VehicleState current_state_;
    PurePursuit& controller_;
    PathTrackingPlanner& planner_;
    std::string& waypoints_file_;


    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_ ;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    // std::string waypoints_file = "/root/catkin_ws/src/POLARIS_GEM_e2-main/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv";
    int waypoint_index = 0;
    geometry_msgs::Twist cmd_msg;
    


    void handleIdleState(std::vector<Waypoint>& global_path_);
    void handlePathFollowingState(std::vector<Waypoint>& global_path_);
    void handleCompletedState();

    bool isPathCompleted();  // Helper function to check if the path is completed
};

#endif // STATE_MACHINE_H