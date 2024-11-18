#ifndef MOCK_STATE_MACHINE_MANAGER_HPP
#define MOCK_STATE_MACHINE_MANAGER_HPP

#include "icd.hpp"
#include <functional>
#include <queue>
#include <optional>
#include <memory>

class MockStateMachineManager
{
public:
    MockStateMachineManager() = default;
    ~MockStateMachineManager() = default;

    // Mock start function
    void start() {}

    // Mock subscription functions
    void subscribeToGpsUpdates(std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> callback)
    {
        m_mockGpsUpdateCallback = std::move(callback);
    }

    void subscribeToLinkUpdates(std::function<void(drone_sdk::SignalQuality)> callback)
    {
        m_mockLinkUpdateCallback = std::move(callback);
    }

    void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback)
    {
        m_mockFlightStateCallback = std::move(callback);
    }

    void subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_mockGpsSignalStateCallback = std::move(callback);
    }

    void subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_mockLinkSignalStateCallback = std::move(callback);
    }

    void subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback)
    {
        m_mockCommandStateCallback = std::move(callback);
    }

    void subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback)
    {
        m_mockWaypointCallback = std::move(callback);
    }

    void subscribeToCurrentDestination(std::function<void(drone_sdk::Location)> callback)
    {
        m_mockCurrentDestinationCallback = std::move(callback);
    }

    void setHome(drone_sdk::Location newHome)
    {
        m_mockHome = std::move(newHome);
    }

    drone_sdk::Location getHome()
    {
        return m_mockHome;
    }

    drone_sdk::FlightControllerStatus newTask(
        drone_sdk::CurrentMission newMission,
        const std::optional<drone_sdk::Location> &singleDestination,
        const std::optional<std::queue<drone_sdk::Location>> &pathDestinations)
    {
        // Simulate task handling
        m_mockCurrentMission = newMission;
        m_mockSingleDestination = singleDestination;
        m_mockPathDestinations = pathDestinations;

        // Return a success status for simplicity
        return drone_sdk::FlightControllerStatus::SUCCESS;
    }

    void handleGpsUpdate(const drone_sdk::Location &location, const drone_sdk::SignalQuality quality)
    {
        if (m_mockGpsUpdateCallback)
            m_mockGpsUpdateCallback(location, quality);
    }

    void handleLinkUpdate(drone_sdk::SignalQuality quality)
    {
        if (m_mockLinkUpdateCallback)
            m_mockLinkUpdateCallback(quality);
    }

    // Mock state machine data
    drone_sdk::CurrentMission getMockCurrentMission() const { return m_mockCurrentMission; }
    std::optional<drone_sdk::Location> getMockSingleDestination() const { return m_mockSingleDestination; }
    std::optional<std::queue<drone_sdk::Location>> getMockPathDestinations() const { return m_mockPathDestinations; }

private:
    // Mock data members
    drone_sdk::Location m_mockHome;
    drone_sdk::CurrentMission m_mockCurrentMission;
    std::optional<drone_sdk::Location> m_mockSingleDestination;
    std::optional<std::queue<drone_sdk::Location>> m_mockPathDestinations;

    // Mock callback functions
    std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> m_mockGpsUpdateCallback;
    std::function<void(drone_sdk::SignalQuality)> m_mockLinkUpdateCallback;
    std::function<void(drone_sdk::FlightState)> m_mockFlightStateCallback;
    std::function<void(drone_sdk::safetyState)> m_mockGpsSignalStateCallback;
    std::function<void(drone_sdk::safetyState)> m_mockLinkSignalStateCallback;
    std::function<void(drone_sdk::CommandStatus)> m_mockCommandStateCallback;
    std::function<void(drone_sdk::Location)> m_mockWaypointCallback;
    std::function<void(drone_sdk::Location)> m_mockCurrentDestinationCallback;
};

#endif // MOCK_STATE_MACHINE_MANAGER_HPP
