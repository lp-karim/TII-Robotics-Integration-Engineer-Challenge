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

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        path_ = *msg;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
        computeControlCommand();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    nav_msgs::Path path_;
    geometry_msgs::Pose current_pose_;

    double lookahead_distance_;
    double max_angular_velocity_;
    double max_linear_velocity_;

    void computeControlCommand(){

        if (path_.poses.empty()) {
            ROS_WARN("No path available.");
            return;
        }

        // Find the lookahead point
        geometry_msgs::PoseStamped lookahead_point = getLookaheadPoint();

        // transform the lookahead point to the vehicle's coordinate frame
        tf2::Transform tf_map_to_base;
        tf2::convert(current_pose_, tf_map_to_base);
        tf2::Transform tf_map_to_lookahead;
        tf2::convert(lookahead_point.pose, tf_map_to_lookahead);
        tf2::Transform tf_base_to_lookahead = tf_map_to_base.inverse() * tf_map_to_lookahead;

        double lx = tf_base_to_lookahead.getOrigin().x();
        double ly = tf_base_to_lookahead.getOrigin().y();

        // calculate the curvature K (kappa)
        double curvature = 2 * ly /(lookahead_distance_ * lookahead_distance_);

        // compute the control commands
        geometry_msgs::Twist cmd;
        cmd.linear.x = max_linear_velocity_;
        cmd.angular.z = curvature * max_angular_velocity_;

        //limit the angular velocity
        if (cmd.angular.z > max_angular_velocity_) {
            cmd.angular.z = max_angular_velocity_;
        }
        else if (cmd.angular.z < -max_angular_velocity_){
            cmd.angular.z = -max_angular_velocity_;
        }

        // publish the control cmd
        cmd_pub_.publish(cmd);
    }
};