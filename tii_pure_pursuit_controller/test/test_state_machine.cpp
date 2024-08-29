#include <gtest/gtest.h>
#include "pure_pursuit_controller/state_machine.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"


std::vector<Waypoint> global_path;

TEST(StateMachineTest, InitialStateIsIdle) {
    ros::NodeHandle nh;
    ros::Publisher dummy_pub = nh.advertise<visualization_msgs::Marker>("dummy_marker", 1);
    PurePursuit dummy_controller(2.0, 2.9, dummy_pub);
    StateMachine sm(dummy_controller);

    ASSERT_EQ(sm.getCurrentState(), VehicleState::Idle);
}

TEST(StateMachineTest, TransitionToPathFollowing) {
    ros::NodeHandle nh;
    ros::Publisher dummy_pub = nh.advertise<visualization_msgs::Marker>("dummy_marker", 1);
    PurePursuit dummy_controller(2.0, 2.9, dummy_pub);
    StateMachine sm(dummy_controller);

    // Simulate a path being available
    global_path.push_back(Waypoint(1.0, 1.0, 0.0));

    sm.run(global_path); // Run the state machine logic once

    ASSERT_EQ(sm.getCurrentState(), VehicleState::PathFollowing);
}

TEST(StateMachineTest, TransitionToCompleted) {
    ros::NodeHandle nh;
    ros::Publisher dummy_pub = nh.advertise<visualization_msgs::Marker>("dummy_marker", 1);
    PurePursuit dummy_controller(2.0, 2.9, dummy_pub);
    StateMachine sm(dummy_controller);

    // Simulate the last waypoint being reached
    global_path.push_back(Waypoint(1.0, 1.0, 0.0));
    sm.run(global_path);  // First run to transition to PathFollowing

    global_path.pop_back(); // Simulate completion of the path
    sm.run(global_path);  // Second run to check transition to Completed

    ASSERT_EQ(sm.getCurrentState(), VehicleState::Completed);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_state_machine");
    return RUN_ALL_TESTS();
}
