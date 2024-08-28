#ifndef FRAME_ID_REMAPPER_H
#define FRAME_ID_REMAPPER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>  // Include the PointCloud2 message type

class FrameIdRemapper
{
public:
    // Constructor
    FrameIdRemapper(ros::NodeHandle& nh, const std::string& input_topic, const std::string& output_topic, const std::string& new_frame_id);

private:
    // Callback function
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Member variables
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string new_frame_id_;
};

#endif // FRAME_ID_REMAPPER_H
