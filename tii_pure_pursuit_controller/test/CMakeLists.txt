# Add test files and link them with the appropriate libraries
catkin_add_gtest(test_state_machine test_state_machine.cpp)
target_link_libraries(test_state_machine ${catkin_LIBRARIES})

catkin_add_gtest(test_pure_pursuit test_pure_pursuit.cpp)
target_link_libraries(test_pure_pursuit ${catkin_LIBRARIES})
