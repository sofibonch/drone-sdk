#include "drone_sdk.hpp"
#include "drone_controller.hpp"
#include <iostream>

DroneSDK::DroneSDK()
    : m_DroneController(std::make_unique<DroneController>()) {}

DroneSDK::DroneSDK(DroneSDK&&) noexcept = default;
DroneSDK& DroneSDK::operator=(DroneSDK&&) noexcept = default;

DroneSDK::~DroneSDK() {
    // Cleanup is handled automatically by the unique_ptr
}

void DroneSDK::emergencyLand() {
    // m_DroneController->emergencyLand(); // Uncomment if needed
}

void DroneSDK::returnHome() {
    m_DroneController->returnHome();
}

void DroneSDK::subscribeToGpsUpdates(drone_sdk::GpsCallback::Type gpsCallback) {
    m_DroneController->getHardwareMonitor().subscribeToGpsUpdates(gpsCallback);
}

void DroneSDK::subscribeToLinkUpdates(drone_sdk::LinkCallback::Type linkCallback) {
    m_DroneController->getHardwareMonitor().subscribeToLinkUpdates(linkCallback);
}

void DroneSDK::start() {
    // Initialize components and start the hardware monitor and state machine manager
}

void DroneSDK::stop() {
    m_DroneController->getHardwareMonitor().stop();  // Stop monitoring and threads
}
