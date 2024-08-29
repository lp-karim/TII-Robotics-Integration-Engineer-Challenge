#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "pure_pursuit_controller/path_tracking_planner.h"
#include "pure_pursuit_controller/frame_id_remapper.h"
#include "pure_pursuit_controller/state_machine.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "path_tracking_node");
//     ros::NodeHandle nh;
    
//     std::string waypoints_file = "/root/catkin_ws/src/POLARIS_GEM_e2-main/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv";
//     nh.param<std::string>("waypoints_file", waypoints_file, waypoints_file);

//     // Parameters for input and output topics, and new frame_id
//     std::string input_topic = "/gem/velodyne_points";
//     std::string output_topic = "/gem/velodyne_points";
//     std::string new_frame_id = "velodyne_base_link";
//     nh.param<std::string>("input_topic", input_topic, input_topic);
//     nh.param<std::string>("output_topic", output_topic, output_topic);
//     nh.param<std::string>("new_frame_id", new_frame_id, new_frame_id);

//     ROS_INFO("Input Topic: %s", input_topic.c_str());
//     ROS_INFO("Output Topic: %s", output_topic.c_str());
//     ROS_INFO("New Frame ID: %s", new_frame_id.c_str());

//     FrameIdRemapper remapper(nh, input_topic, output_topic, new_frame_id);

//     // Pure Pursuit setup
//     double lookahead_distance = 6.0;  // Can be parameterized
//     double wheelbase = 1.5;  // Example value, adjust based on your robot

//     nh.param<double>("lookahead_distance", lookahead_distance, lookahead_distance);
//     nh.param<double>("wheelbase", wheelbase, wheelbase);

//     // Pure Pursuit controller instance
//     PurePursuit pure_pursuit(lookahead_distance, wheelbase, marker_pub);

//     // Path Tracking Planner setup
//     PathTrackingPlanner planner(nh);
//     std::vector<Waypoint> global_path_ = planner.loadWaypointsFromCSV(waypoints_file);
    
//     ros::Rate loop_rate(10);  // 10 Hz
//     while (ros::ok()) {
//         if (!global_path_.empty()) {
//             std::vector<Waypoint> transformed_path = planner.transformGlobalPathToVehicleFrame(global_path_, planner.getCurrentPosition());
//             double steering_angle = pure_pursuit.computeSteeringAngle(planner.getCurrentPosition(), transformed_path);
//             planner.publishPath(global_path_, path_pub_);

//             geometry_msgs::Twist cmd_msg;
//             cmd_msg.linear.x = 1.0;  // Example speed, adjust as needed
//             cmd_msg.angular.z = steering_angle;

//             cmd_pub_.publish(cmd_msg);
//         }
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }

// Global variable for waypoints
std::vector<Waypoint> global_path_;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);
    ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("/gem/path", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    std::string waypoints_file_;
    nh.param<std::string>("waypoints_file", waypoints_file_, "/root/catkin_ws/src/POLARIS_GEM_e2-main/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv");
    // Parameters for input and output topics, and new frame_id
    std::string input_topic = "/gem/velodyne_points";
    std::string output_topic = "/gem/velodyne_points";
    std::string new_frame_id = "velodyne_base_link";
    nh.param<std::string>("input_topic", input_topic, input_topic);
    nh.param<std::string>("output_topic", output_topic, output_topic);
    nh.param<std::string>("new_frame_id", new_frame_id, new_frame_id);

    // ROS_INFO("Input Topic: %s", input_topic.c_str());
    // ROS_INFO("Output Topic: %s", output_topic.c_str());
    // ROS_INFO("New Frame ID: %s", new_frame_id.c_str());

    FrameIdRemapper remapper(nh, input_topic, output_topic, new_frame_id);

    PurePursuit pure_pursuit(2.0, 1.5, marker_pub);
    PathTrackingPlanner planner(nh);
    StateMachine state_machine(pure_pursuit,planner,waypoints_file_);

    global_path_ = planner.loadWaypointsFromCSV(waypoints_file_);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        state_machine.run(global_path_);  // Run the state machine to handle the robot's behavior

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}