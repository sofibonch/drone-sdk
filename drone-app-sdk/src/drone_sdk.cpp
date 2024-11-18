#include "drone_sdk.hpp"
#include <iostream>

DroneSDK::DroneSDK()
    : m_DroneController(std::make_unique<DroneController>()) {}

DroneSDK::DroneSDK(DroneSDK&&) noexcept = default;

DroneSDK& DroneSDK::operator=(DroneSDK&&) noexcept = default;

DroneSDK::~DroneSDK() {
    // Cleanup is handled automatically by the unique_ptr
}

drone_sdk::FlightControllerStatus DroneSDK::goTo(const drone_sdk::Location &location)
{
    // Forward the command to DroneController
    return m_DroneController->goTo(location);
}

drone_sdk::FlightControllerStatus DroneSDK::abortMission()
{
    // Forward the command to DroneController
    return m_DroneController->abortMission();
}

drone_sdk::FlightControllerStatus DroneSDK::hover()
{
    // Forward the command to DroneController
    return m_DroneController->hover();
}

//void DroneSDK::start() {
//   
//}
//
//void DroneSDK::stop() {
//
//}
