#!/bin/bash
set -e

# Source the ROS setup
source /opt/ros/noetic/setup.bash

# Ensure the workspace directory is set up correctly
WORKSPACE_DIR="/root/catkin_ws"

# Navigate to the workspace directory
cd $WORKSPACE_DIR

# Debug: Print the current directory and list its contents
echo "Current directory: $(pwd)"
echo "Contents:"
ls -la

# Initialize the catkin workspace if necessary
if [ ! -f "$WORKSPACE_DIR/src/CMakeLists.txt" ]; then
  echo "Initializing workspace"
  catkin_init_workspace src
fi

# Build the catkin workspace
catkin_make

# Source the workspace setup
source $WORKSPACE_DIR/devel/setup.bash

# Allow Docker containers to use your display
xhost +local:root

# Print a message to indicate the container is ready
echo "Container is ready and running. You are now in the bash shell."

# Start an interactive bash shell
# exec bash
# source devel/setup.bash && roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"

tail -f /dev/null   