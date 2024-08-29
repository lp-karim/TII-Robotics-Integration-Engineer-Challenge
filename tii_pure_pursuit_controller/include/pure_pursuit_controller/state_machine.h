#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "ros/ros.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"

enum class VehicleState {
    Idle,
    PathFollowing,
    Completed
};