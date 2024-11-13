#include "drone_sdk.hpp"
#include <iostream>

DroneSDK::DroneSDK() 
    : m_hwMonitor(), m_stateMachineManager(m_hwMonitor) {
    // Initialize the state machine manager which will manage the safety state machine
    m_hwMonitor.start();
    m_stateMachineManager.start();
}

DroneSDK::~DroneSDK() {
    stop();  // Ensure to stop hardware monitor and threads on destruction
}

void DroneSDK::start() {
    // Start the hardware monitor (polling GPS, Link, etc.)
    m_hwMonitor.start();
}

void DroneSDK::stop() {
    // Stop the hardware monitor to cleanly stop all threads
    m_hwMonitor.stop();
}

void DroneSDK::subscribeToGpsUpdates(drone_sdk::GpsCallback::Type gpsCallback) {
    // Subscribe to GPS updates from the HardwareMonitor
    m_hwMonitor.subscribeToGpsUpdates([this, gpsCallback](auto location, auto signalQuality) {
        // Call the provided callback function with the ICD Location and SignalQuality
        gpsCallback(location, signalQuality);
    });
}

void DroneSDK::subscribeToLinkUpdates(drone_sdk::LinkCallback::Type linkCallback) {
    // Subscribe to Link updates from the HardwareMonitor
    m_hwMonitor.subscribeToLinkUpdates([this, linkCallback](auto signalQuality) {
        // Call the provided callback function with the ICD SignalQuality
        linkCallback(signalQuality);
    });
}
void DroneSDK::emergencyLand() {
    std::cout << "Emergency Landing Triggered" << std::endl;
}

void DroneSDK::returnHome() {
    std::cout << "Return Home Triggered" << std::endl;
}
