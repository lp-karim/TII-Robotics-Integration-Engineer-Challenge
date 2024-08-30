# Base image with ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/root/catkin_ws

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-tf \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-gazebo-ros \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Copy the built application from the dev container
COPY --from=ros_dev /root/catkin_ws /root/catkin_ws

# Source the workspace and run the application
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source $ROS_WS/devel/setup.bash" >> ~/.bashrc
