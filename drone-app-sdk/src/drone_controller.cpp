#include "drone_controller.hpp"
#include <iostream>

DroneController::DroneController()
    : m_stateMachineManager(m_hwMonitor) {
    m_hwMonitor.start();  // Start monitoring
    m_stateMachineManager.start();  // Start state machine
}

DroneController::~DroneController() {
    m_hwMonitor.stop();  // Clean up resources
}

void DroneController::hover() {
    std::cout << "Hover command executed." << std::endl;
    // Additional logic for the hover command can go here.
}

void DroneController::emergencyLand() {
    std::cout << "Emergency Land command executed." << std::endl;
    // Additional logic for the emergency land command can go here.
}

void DroneController::returnHome() {
    std::cout << "Return Home command executed." << std::endl;
    // Additional logic for the return home command can go here.
}
