#include "pure_pursuit_controller/frame_id_remapper.h"

FrameIdRemapper::FrameIdRemapper(ros::NodeHandle& nh, const std::string& input_topic, const std::string& output_topic, const std::string& new_frame_id)
    : new_frame_id_(new_frame_id)
{
    // Initialize the subscriber and publisher
    sub_ = nh.subscribe(input_topic, 10, &FrameIdRemapper::callback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
}

void FrameIdRemapper::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Copy the message
    sensor_msgs::PointCloud2 new_msg = *msg;

    // Modify the frame_id
    new_msg.header.frame_id = new_frame_id_;

    // Publish the modified message
    pub_.publish(new_msg);
}
