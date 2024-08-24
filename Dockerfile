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
    ros-noetic-ackermann-msgs \ 
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \ 
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \ 
    ros-noetic-ros-controllers \ 
    ros-noetic-velodyne-simulator \
    && rm -rf /var/lib/apt/lists/*

# Add X11 dependencies
RUN apt-get update && apt-get install -y \
    libx11-dev \
    x11-xserver-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libqt5widgets5 \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Copy the current directory into the container at /root/catkin_ws/src
COPY . /root/catkin_ws/src/

# Build the catkin workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Copy the entrypoint script
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Set up the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]
