#include "ros/ros.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "pure_pursuit_controller/path_tracking_planner.h"
#include "pure_pursuit_controller/state_machine.h"


StateMachine::StateMachine(PurePursuit& controller, PathTrackingPlanner& planner, std::string& waypoints_file, std::vector<Waypoint>& global_path_)
    : current_state_(VehicleState::Idle), controller_(controller), planner_(planner), waypoints_file_(waypoints_file)  {

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/gem/path", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        remaing_waypoint = global_path_.size();
    }

void StateMachine::run(std::vector<Waypoint>& global_path_) {
    
    planner_.publishPath(global_path_, path_pub_);
    switch (current_state_) {
        case VehicleState::Idle:
            handleIdleState(global_path_);
            break;
        case VehicleState::PathFollowing:
            handlePathFollowingState(global_path_);
            break;
        case VehicleState::Completed:
            handleCompletedState(global_path_);
            break;
    }
}

void StateMachine::handleIdleState(std::vector<Waypoint>& global_path_) {
    // ANSI escape code for Yellow
    const std::string reset   = "\033[0m";
    const std::string yellow  = "\033[33m";
    ROS_INFO("%sState: Idle. Waiting for path...%s",yellow.c_str(), reset.c_str());

    // Transition to PathFollowing state if path is available
    if (!global_path_.empty()) {

        current_state_ = VehicleState::PathFollowing;
    }
}

void StateMachine::handlePathFollowingState(std::vector<Waypoint>& global_path_) {
    if ((remaing_waypoint - waypoint_index ) >  20)   {
        // ANSI escape code for Blue
        const std::string reset   = "\033[0m";
        const std::string blue    = "\033[34m";
        ROS_INFO("%sState: PathFollowing. Tracking the path...%s",blue.c_str(), reset.c_str());
        
        // Compute the steering angle
        std::pair<double, int> result = controller_.computeSteeringAngle(planner_.getCurrentPosition(), global_path_);
        double steering_angle = result.first;
        waypoint_index = result.second;

        // Publish the command
        cmd_msg.linear.x = 2.5;  // 1.0
        cmd_msg.angular.z = steering_angle;

        cmd_pub_.publish(cmd_msg);
    }
    // If the vehicle has reached the last waypoint, transition to Completed state
    else  {
        global_path_.clear();
        // ANSI escape code for Green
        const std::string reset   = "\033[0m";
        const std::string green   = "\033[32m";
        ROS_INFO("%sReached the last waypoint. Transitioning to Completed state.%s",green.c_str(), reset.c_str());
        current_state_ = VehicleState::Completed;
        return;
    }
    ROS_INFO("Remaining Waypoints: %d", (remaing_waypoint - waypoint_index ));
    //  else {
    //     remaing_waypoint -= waypoint_index;
    // }
}

void StateMachine::handleCompletedState(std::vector<Waypoint>& global_path_) {
    // ANSI escape code for Green
    const std::string reset   = "\033[0m";
    const std::string green   = "\033[32m";
    ROS_INFO("%sState: Completed. Stopping The Vehicle.%s",green.c_str(), reset.c_str());

    //stopping the Vehicle
    waypoint_index = 0;
    cmd_msg.linear.x = 0.0;  
    cmd_msg.angular.z = 0.0;

    cmd_pub_.publish(cmd_msg);
}

VehicleState StateMachine::getCurrentState() const {
    return current_state_;
}