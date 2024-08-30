# TII-Robotics-Integration-Engineer-Challenge
## ROS Path-Tracking Controller with State Machine

This project implements a ROS-based path-tracking controller using a Pure Pursuit algorithm and a navigation state machine. The project is containerized using Docker, with separate development and production environments managed via Docker Compose.

## Features

- **Path-Tracking Controller**: Implements a Pure Pursuit algorithm to follow a specified path.
- **State Machine**: Manages robot states, including Idle, Path Following, and Completed.
- **Docker Compose**: Two containers:
  - **dev**: For building and developing the ROS package.
  - **app**: For running the compiled ROS application.
- **X11 Forwarding**: Supports running GUI applications like RViz within the Docker containers.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)
- X11 server running on your host machine (for GUI applications)

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/lp-karim/TII-Robotics-Integration-Engineer-Challenge
cd TII-Robotics-Integration-Engineer-Challenge
```

### 2. Build the Docker Containers

Use Docker Compose to build the containers:

```bash

docker compose up --build -d
```
### 3. Running GUI Applications

Make sure to expose your X11 socket to be able to Rviz and Gazebo:

```bash

xhost +local:docker
```
## Achievements


## Further Development

- **Unit Testing**: Use gtest to write and run unit tests for each component.
- **Code Coverage**: Use gcov or lcov to measure code coverage and aim for 80% coverage.
- **Extend State Machine**: Add more states to the state machine, such as obstacle avoidance.