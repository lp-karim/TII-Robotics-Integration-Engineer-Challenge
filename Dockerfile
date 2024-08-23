# Use an official ROS image as a base
FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    ros-noetic-nav-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Copy the current directory into the container at /src
COPY . /root/catkin_ws/src/

# Build the catkin workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set up the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
