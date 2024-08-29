#include "ros/ros.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "pure_pursuit_controller/path_tracking_planner.h"
#include "pure_pursuit_controller/state_machine.h"

// global variable
// extern std::vector<Waypoint> global_path_;


StateMachine::StateMachine(PurePursuit& controller, PathTrackingPlanner& planner, std::string& waypoints_file)
    : current_state_(VehicleState::Idle), controller_(controller), planner_(planner), waypoints_file_(waypoints_file)  {

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/gem/path", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

void StateMachine::run(std::vector<Waypoint>& global_path_) {
    // global_path_ = planner_.loadWaypointsFromCSV(waypoints_file_);
    ROS_INFO("Waypoints loaded: %lu", global_path_.size());
    planner_.publishPath(global_path_, path_pub_);
    // for (size_t i = 0; i < global_path_.size(); ++i) {
    //         ROS_INFO("Waypoint %lu: x = %f, y = %f, yaw = %f", i, global_path_[i].x, global_path_[i].y, global_path_[i].yaw);
    //     }
    switch (current_state_) {
        case VehicleState::Idle:
            handleIdleState(global_path_);
            break;
        case VehicleState::PathFollowing:
            handlePathFollowingState(global_path_);
            break;
        case VehicleState::Completed:
            handleCompletedState();
            break;
    }
}

void StateMachine::handleIdleState(std::vector<Waypoint>& global_path_) {
    ROS_INFO("State: Idle. Waiting for path...");

    // Transition to PathFollowing state if path is available
    if (!global_path_.empty()) {

        current_state_ = VehicleState::PathFollowing;
    }
}

void StateMachine::handlePathFollowingState(std::vector<Waypoint>& global_path_) {
    ROS_INFO("State: PathFollowing. Tracking the path...");

    // Check if waypoint_index has reached the last waypoint
    if (waypoint_index >= global_path_.size() - 1) {
        ROS_INFO("Reached the last waypoint. Transitioning to Completed state.");
        current_state_ = VehicleState::Completed;
        return;
    }

    // Transform the global path into the vehicle's coordinate frame
    std::vector<Waypoint> transformed_path = planner_.transformGlobalPathToVehicleFrame(global_path_, planner_.getCurrentPosition());
    planner_.publishPath(global_path_, path_pub_);

    // Compute the steering angle
    std::pair<double, int> result = controller_.computeSteeringAngle(planner_.getCurrentPosition(), transformed_path);
    double steering_angle = result.first;
    waypoint_index = result.second;

    // Publish the command
    cmd_msg.linear.x = 1.0;  // 1.0
    cmd_msg.angular.z = steering_angle;

    cmd_pub_.publish(cmd_msg);

    // If the vehicle has reached the last waypoint, transition to Completed state
    if (waypoint_index >= global_path_.size() - 1) {
        ROS_INFO("Reached the last waypoint. Transitioning to Completed state.");
        current_state_ = VehicleState::Completed;
    }
}

void StateMachine::handleCompletedState() {
    ROS_INFO("State: Completed. Stopping The Vehicle.");

    //stopping the Vehicle
    waypoint_index = 0;
    cmd_msg.linear.x = 0.0;  
    cmd_msg.angular.z = 0.0;

    cmd_pub_.publish(cmd_msg);
}

bool StateMachine::isPathCompleted() {
    // This could be based on the distance to the last waypoint or some other condition
    // Placeholder logic:
    // return global_path_.size() == 1;  // Example condition
}