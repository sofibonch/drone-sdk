#include "drone_controller.hpp"
#include <iostream>

DroneController::DroneController()
    : m_stateMachineManager()
    , m_commandController () {
    m_hwMonitor.start();  // Start monitoring
    m_stateMachineManager.start(&m_hwMonitor);  // Start state machine
    m_commandController.start(&m_stateMachineManager);
}

DroneController::~DroneController() {
    m_hwMonitor.stop();  // Clean up resources
}


bool DroneController::goTo(const drone_sdk::Location & location){
    return m_commandController.goTo(location);
}
//bool DroneController::followPath(const std::vector<drone_sdk::Location> &path){
//    return m_commandController.followPath(path);
//}
bool DroneController::abortMission(){
    return m_commandController.abortMission();
}
bool  DroneController::hover() {
    std::cout << "Hover command executed." << std::endl;
    return m_commandController.hover();
}
