#include "drone_sdk.hpp"

DroneSDK::DroneSDK()
    : m_DroneController(std::make_unique<DroneController>()) // Initialize DroneController
{
}

DroneSDK::DroneSDK(DroneSDK &&) noexcept = default;
DroneSDK &DroneSDK::operator=(DroneSDK &&) noexcept = default;

DroneSDK::~DroneSDK() = default;

drone_sdk::FlightControllerStatus DroneSDK::goTo(const drone_sdk::Location &location)
{
    return m_DroneController->goTo(location);
}

drone_sdk::FlightControllerStatus DroneSDK::abortMission()
{
    return m_DroneController->abortMission();
}

drone_sdk::FlightControllerStatus DroneSDK::hover()
{
    return m_DroneController->hover();
}

drone_sdk::FlightControllerStatus DroneSDK::path(std::queue<drone_sdk::Location> locations)
{
    return m_DroneController->path(locations);
}

void DroneSDK::subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback)
{
    m_DroneController->subscribeToGpsSignalState(callback);
}

void DroneSDK::subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback)
{
    m_DroneController->subscribeToLinkSignalState(callback);
}

void DroneSDK::subscribeToGpsLocation(std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> callback)
{
    m_DroneController->subscribeToGpsLocation(callback);
}

void DroneSDK::subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback)
{
    m_DroneController->subscribeToFlightState(callback);
}

void DroneSDK::subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback)
{
    m_DroneController->subscribeToCommandState(callback);
}

void DroneSDK::subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback)
{
    m_DroneController->subscribeToWaypoint(callback);
}
