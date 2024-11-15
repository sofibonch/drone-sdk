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

bool DroneSDK::goTo(const drone_sdk::Location & location){
    return m_DroneController->goTo(location);
}

//bool DroneSDK::followPath(const std::vector<drone_sdk::Location> &path){
//    return m_DroneController->followPath(path);
//}
bool DroneSDK::abortMission(){
    return m_DroneController->abortMission();
}
bool DroneSDK::hover(){
    return m_DroneController->hover();
}

//void DroneSDK::subscribeToGpsUpdates(drone_sdk::GpsCallback::Type gpsCallback) {
//    m_DroneController.subscribeToGpsUpdates(gpsCallback);
//}
//
//void DroneSDK::subscribeToLinkUpdates(drone_sdk::LinkCallback::Type linkCallback) {
//    m_DroneController.subscribeToLinkUpdates(linkCallback);
//}

void DroneSDK::start() {
   
}

void DroneSDK::stop() {
    //m_DroneController.stop();  // Stop monitoring and threads
}
