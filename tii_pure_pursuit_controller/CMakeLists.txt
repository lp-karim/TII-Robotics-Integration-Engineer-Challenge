cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_controller)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare the C++ executable
add_executable(${PROJECT_NAME}_node src/main.cpp src/pure_pursuit_controller.cpp)

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)