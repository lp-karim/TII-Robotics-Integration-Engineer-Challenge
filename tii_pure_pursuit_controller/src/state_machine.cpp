#include "ros/ros.h"
#include "pure_pursuit_controller/pure_pursuit_controller.h"
#include "state_machine.h"



StateMachine::StateMachine(PurePursuit& controller)
    : current_state_(VehicleState::Idle), controller_(controller) {}

void StateMachine::run() {
    switch (current_state_) {
        case VehicleState::Idle:
            handleIdleState();
            break;
        case VehicleState::PathFollowing:
            handlePathFollowingState();
            break;
        case VehicleState::Completed:
            handleCompletedState();
            break;
    }
}