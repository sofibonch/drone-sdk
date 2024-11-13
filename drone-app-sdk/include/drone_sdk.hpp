#ifndef DRONE_SDK_HPP
#define DRONE_SDK_HPP

#include "hw_monitor.hpp"            // For HardwareMonitor
#include "state_machine_manager.hpp" // For StateMachineManager
#include "icd.hpp"                   // For callback types

class DroneSDK {
public:
    DroneSDK();
    ~DroneSDK();

    // User commands (e.g., emergency stop, return home, etc.)
    void emergencyLand();
    void returnHome();

    // Subscribe to GPS and Link updates with callback functions
    void subscribeToGpsUpdates(drone_sdk::GpsCallback::Type gpsCallback);
    void subscribeToLinkUpdates(drone_sdk::LinkCallback::Type linkCallback);
    
private:
    void start();  // Initialize the system and start the monitoring threads
    void stop();   // Clean up resources and stop threads

private:
    HardwareMonitor m_hwMonitor;              // HardwareMonitor instance
    StateMachineManager m_stateMachineManager; // StateMachineManager instance
};
#endif // DRONE_SDK_HPP
