#include <gtest/gtest.h>
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "ros/ros.h"

TEST(PurePursuitTest, ComputeSteeringAngle) {
    ros::NodeHandle nh;
    ros::Publisher dummy_pub = nh.advertise<visualization_msgs::Marker>("dummy_marker", 1);
    PurePursuit pp(2.0, 2.9, dummy_pub);

    Waypoint current_position(0.0, 0.0, 0.0);
    std::vector<Waypoint> path = {
        Waypoint(2.0, 2.0, 0.0),
        Waypoint(4.0, 4.0, 0.0)
    };

    double steering_angle = pp.computeSteeringAngle(current_position, path);

    // Assert that the steering angle is within an expected range
    ASSERT_NEAR(steering_angle, 0.0, 0.1);  // Example, adjust the range as needed
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_pure_pursuit");
    return RUN_ALL_TESTS();
}
