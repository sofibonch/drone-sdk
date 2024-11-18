#ifndef STATE_MACHINE_MANAGER_HPP
#define STATE_MACHINE_MANAGER_HPP

#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"
#include "state_machines/command_state_machine.hpp"
#include "icd.hpp"

#include <boost/sml.hpp>
#include <queue>
#include <optional>
#include <functional>
#include <memory>

class StateMachineManager
{
public:
    explicit StateMachineManager() = default;
    ~StateMachineManager() = default;
    void start()
    {
        m_safetySM.subscribeToGpsState([this](drone_sdk::safetyState gpsState)
                                       { m_commandSM.handleGpsStateChange(gpsState); });

        m_safetySM.subscribeToLinkState([this](drone_sdk::safetyState linkState)
                                        { m_commandSM.handleLinkStateChange(linkState); });

        // subscribe flight state machine to commands
        m_commandSM.subscribeToState([this](drone_sdk::CommandStatus commandState)
                                     { m_flightSM.handleCommandStateChange(commandState); });
    }
    // Subscription functions for external components
    void subscribeToGpsUpdates(std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> callback)
    {
        m_gpsUpdateCallback = std::move(callback);
    }

    void subscribeToLinkUpdates(std::function<void(drone_sdk::SignalQuality)> callback)
    {
        m_linkUpdateCallback = std::move(callback);
    }

    // State subscriptions
    void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback)
    {
        m_flightSM.subscribeToStateChange(std::move(callback));
    }

    void subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_safetySM.subscribeToGpsState(std::move(callback));
    }

    void subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_safetySM.subscribeToLinkState(std::move(callback));
    }

    void subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback)
    {
        m_commandSM.subscribeToState(std::move(callback));
    }

    void subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback)
    {
        m_commandSM.subscribeToPathWaypoint(std::move(callback));
    }

    void setHome(drone_sdk::Location newHome)
    {
        m_commandSM.setHomebase(std::move(newHome));
    }

    drone_sdk::Location getHome()
    {
        return m_commandSM.getHomebase();
    }

    void subscribeToCurrentDestination(std::function<void(drone_sdk::Location)> callback)
    {
        m_commandSM.subscribeToCurrentDestination(std::move(callback));
    }

    drone_sdk::FlightControllerStatus newTask(
        drone_sdk::CurrentMission newMission,
        const std::optional<drone_sdk::Location> &singleDestination,
        const std::optional<std::queue<drone_sdk::Location>> &pathDestinations)
    {
        return m_commandSM.handleTaskAssigned(newMission, singleDestination, pathDestinations);
    }

    void handleGpsUpdate(const drone_sdk::Location &location, const drone_sdk::SignalQuality quality)
    {
        if (m_gpsUpdateCallback)
            m_gpsUpdateCallback(location, quality);

        m_safetySM.handleGpsSignal(quality);
        m_commandSM.handleGpsLocationUpdate(location);
    }

    void handleLinkUpdate(drone_sdk::SignalQuality quality)
    {
        if (m_linkUpdateCallback)
            m_linkUpdateCallback(quality);

        m_safetySM.handleLinkSignal(quality);
    }

private:
    flightstatemachine::FlightStateMachine m_flightSM;
    safetystatemachine::SafetyStateMachine m_safetySM;
    commandstatemachine::CommandStateMachine m_commandSM;

    // Callback functions for GPS and Link updates
    std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> m_gpsUpdateCallback;
    std::function<void(drone_sdk::SignalQuality)> m_linkUpdateCallback;
};

#endif // STATE_MACHINE_MANAGER_HPP
