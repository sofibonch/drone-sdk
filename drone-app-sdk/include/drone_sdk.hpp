#ifndef DRONE_SDK_HPP
#define DRONE_SDK_HPP

#include "drone_controller.hpp" // For DroneController
#include "icd.hpp"
class DroneSDK
{
public:
    DroneSDK();
    DroneSDK(DroneSDK &&) noexcept;
    DroneSDK &operator=(DroneSDK &&) noexcept;
    ~DroneSDK();

    drone_sdk::FlightControllerStatu goTo(const drone_sdk::Location &location);
    //   bool followPath(const std::vector<drone_sdk::Location> &path);
    drone_sdk::FlightControllerStatu abortMission();
    drone_sdk::FlightControllerStatu hover();

    // Subscribe to GPS and Link updates with callback functions
    //    void subscribeToGpsUpdates(drone_sdk::GpsCallback::Type gpsCallback);
    //    void subscribeToLinkUpdates(drone_sdk::LinkCallback::Type linkCallback);

private:
    //void start(); // Initialize the system and start the monitoring threads
    //void stop();  // Clean up resources and stop threads
    std::unique_ptr<DroneController> m_DroneController;
};

#endif // DRONE_SDK_HPP
