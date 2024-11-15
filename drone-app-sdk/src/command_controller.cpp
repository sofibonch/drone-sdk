#include "command_controller.hpp"
#include <iostream>
void CommandController::start(StateMachineManager * stateMachineManager)
{
    m_stateMachineManager=stateMachineManager;
}
// Command to go to a specified location
bool CommandController::goTo(const drone_sdk::Location& location) {
    if (m_cur_flight_state == drone_sdk::FlightState::LANDED) {
        // Construct an Altitude object using location.altitude
        drone_sdk::Altitude alt{location.altitude};  // Use braces if Altitude has a single-field constructor
        if (m_flightControllerHandler.arm() != drone_sdk::FlightControllerStatus::SUCCESS ||
            m_flightControllerHandler.takeOff(alt) != drone_sdk::FlightControllerStatus::SUCCESS) {
            std::cerr << "Failed to arm or take off." << std::endl;
            return false;
        }
    }

    // Go directly to the specified location
    return m_flightControllerHandler.goTo(location) == drone_sdk::FlightControllerStatus::SUCCESS;
}


// Command to follow a path of multiple locations using queue
//bool CommandController::followPath(const std::queue<drone_sdk::Location>& path) {
   // m_pathQueue = path;  // Load the queue with path points
   // if (!m_pathQueue.empty()) {
   //     return goTo(m_pathQueue.front());  // Start with the first location in the queue
   // }
//    return false;
//}

// Command to abort mission and return to home base
bool CommandController::abortMission() {

    if (m_flightControllerHandler.goTo(m_homebase) != drone_sdk::FlightControllerStatus::SUCCESS) {
        std::cerr << "Failed to abort mission and return to home base." << std::endl;
        return false;
    }
    return true;
}

// Command to hover at the current location
bool CommandController::hover() {
    if (m_cur_flight_state == drone_sdk::FlightState::TAKEOFF ||
        m_cur_flight_state == drone_sdk::FlightState::AIRBORNE ||
        m_cur_flight_state == drone_sdk::FlightState::HOVER) {
        if (m_flightControllerHandler.goTo(m_cur_location) != drone_sdk::FlightControllerStatus::SUCCESS) {
            std::cerr << "Failed to hover at current location." << std::endl;
            return false;
        }
        return true;
    }
    std::cerr << "Cannot hover. Current flight state does not permit hovering." << std::endl;
    return false;
}
