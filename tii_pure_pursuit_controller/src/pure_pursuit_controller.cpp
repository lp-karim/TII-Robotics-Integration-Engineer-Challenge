#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh) : nh_(nh) {
        path_sub_ = nh_.subscribe("path", 1000, &PurePursuitController::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 1000, &PurePursuitController::odometryCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        nh_.param("lookahead_distance", lookahead_distance_, 1.0);
        nh_.param("max_linear_velocity", max_linear_velocity_, 2.0);
        nh_.param("max_angular_velocity", max_angular_velocity_, 1.0);
    }



};