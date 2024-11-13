#ifndef DRONE_SDK_HPP
#define DRONE_SDK_HPP

#include "drone_controller.hpp" // For DroneController
#include "icd.hpp"
class DroneSDK {
public:
    DroneSDK();
    DroneSDK(DroneSDK&&) noexcept;
    DroneSDK& operator=(DroneSDK&&) noexcept;
    ~DroneSDK();

    // User commands (e.g., emergency stop, return home, etc.)
    void emergencyLand();
    void returnHome();

    // Subscribe to GPS and Link updates with callback functions
    void subscribeToGpsUpdates(drone_sdk::GpsCallback::Type gpsCallback);
    void subscribeToLinkUpdates(drone_sdk::LinkCallback::Type linkCallback);

    void start();  // Initialize the system and start the monitoring threads
    void stop();   // Clean up resources and stop threads

private:
    std::unique_ptr<DroneController> m_DroneController;
};

#endif // DRONE_SDK_HPP
