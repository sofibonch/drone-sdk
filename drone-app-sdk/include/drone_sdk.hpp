#ifndef DRONE_SDK_HPP
#define DRONE_SDK_HPP

#include "drone_controller.hpp" // For DroneController
#include "icd.hpp"              // For various drone-related types (Location, SignalQuality, etc.)
#include <memory>               // For smart pointers

/**
 * @brief The DroneSDK class is responsible for managing the drone's actions and states.
 * It abstracts the operations of the drone, including commanding it to go to specific locations,
 * aborting missions, and subscribing to various telemetry data.
 */
class DroneSDK
{
public:
    DroneSDK();
    ~DroneSDK() = default;

    DroneSDK(const DroneSDK &) = delete;
    DroneSDK &operator=(const DroneSDK &) = delete;
    DroneSDK(DroneSDK &&) noexcept = default;
    DroneSDK &operator=(DroneSDK &&) noexcept = default;

    /**
     * @brief Commands the drone to move to the specified location.
     * @param location The target location to which the drone should go.
     * @retval FlightControllerStatus The status of the flight ).
     */
    drone_sdk::FlightControllerStatus goTo(const drone_sdk::Location &location);

    /**
     * @brief Commands the drone to abort the current mission.
     * @retval FlightControllerStatus The status of the abort ).
     */
    drone_sdk::FlightControllerStatus abortMission();

    /**
     * @brief Commands the drone to hover in its current position.
     * @retval FlightControllerStatus The status of the hover ).
     */
    drone_sdk::FlightControllerStatus hover();

    /**
     * @brief Commands the drone to follow a path consisting of multiple locations.
     * @param locations A queue of locations that the drone will follow in sequence.
     * @retval FlightControllerStatus The status of the first destination in path.
     */
    drone_sdk::FlightControllerStatus path(std::queue<drone_sdk::Location> locations);

    /**
     * @brief Subscribes to GPS signal state changes.
     * @param callback A callback function that will be invoked when the GPS signal state changes.
     */
    void subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback);

    /**
     * @brief Subscribes to link signal state changes.
     * @param callback A callback function that will be invoked when the link signal state changes.
     */
    void subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback);

    /**
     * @brief Subscribes to GPS location updates.
     * @param callback A callback function that will be invoked when a new GPS location is available.
     * @note The callback receives both location and signal quality.
     */
    void subscribeToGpsLocation(std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> callback);

    /**
     * @brief Subscribes to flight state changes.
     * @param callback A callback function that will be invoked when the flight state changes.
     */
    void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback);

    /**
     * @brief Subscribes to command state changes.
     * @param callback A callback function that will be invoked when the command state changes.
     */
    void subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback);

    /**
     * @brief Subscribes to waypoint updates.
     * @param callback A callback function that will be invoked when a new waypoint is reached or set.
     */
    void subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback);

private:
    // Unique pointer to the DroneController object. The controller manages the drone's actions and states.
    std::unique_ptr<DroneController> m_DroneController;
};

#endif // DRONE_SDK_HPP
