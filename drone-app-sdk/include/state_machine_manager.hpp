#ifndef STATE_MACHINE_MANAGER_HPP
#define STATE_MACHINE_MANAGER_HPP

#include "hw_monitor.hpp"
#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"
#include "state_machines/command_state_machine.hpp"

#include "icd.hpp"
#include <boost/sml.hpp>
#include <queue>
#include <optional>
#include <iostream>
#include <memory>

class StateMachineManager
{
public:
    explicit StateMachineManager() = default;
    ~StateMachineManager() = default;

    void start(HardwareMonitor *hwMonitor)
    {
        hwMonitor->subscribeToGpsUpdates([this](const drone_sdk::Location &location, const drone_sdk::SignalQuality quality)
                                         {
                                             m_safetySM.handleGpsSignal(quality);           // Ensure proper mapping
                                             m_commandSM.handleGpsLocationUpdate(location); // Update Command State Machine
                                         });

        // Subscribe to Link updates
        hwMonitor->subscribeToLinkUpdates([this](drone_sdk::SignalQuality quality)
                                          { m_safetySM.handleLinkSignal(quality); });

        // subscribe the command state machine to safety cancerns
        m_safetySM.subscribeToGpsState([this](drone_sdk::safetyState gpsState)
                                       { m_commandSM.handleGpsStateChange(gpsState); });

        m_safetySM.subscribeToLinkState([this](drone_sdk::safetyState linkState)
                                        { m_commandSM.handleLinkStateChange(linkState); });

        // subscribe flight state machine to commands
        m_commandSM.subscribeToState([this](drone_sdk::CommandStatus commandState)
                                     { m_flightSM.handleCommandStateChange(commandState); });
    }

    void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback)
    {
        m_flightSM.subscribeToStateChange(callback);
    }

    void subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_safetySM.subscribeToGpsState(callback);
    }

    void subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_safetySM.subscribeToLinkState(callback);
    }

    void subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback)
    {
        m_commandSM.subscribeToState(callback);
    }

    void subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback)
    {
        m_commandSM.subscribeToPathWaypoint(callback);
    }

    void setHome(drone_sdk::Location newHome)
    {
        m_commandSM.setHomebase(newHome);
    }

    drone_sdk::Location getHome()
    {
        return m_commandSM.getHomebase();
    }

    void subscribeToCurrentDestination(std::function<void(drone_sdk::Location)> callback)
    {
        m_commandSM.subscribeToCurrentDestination(callback);
    }

    drone_sdk::FlightControllerStatus newTask(
        drone_sdk::CurrentMission newMission,
        const std::optional<drone_sdk::Location> &singleDestination,
        const std::optional<std::queue<drone_sdk::Location>> &pathDestinations)
    {
        // Handle task assignment based on available parameters
        return m_commandSM.handleTaskAssigned(newMission, singleDestination, pathDestinations);
    }

private:
    void subscribeToGpsState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_safetySM.subscribeToGpsState(callback);
    }

    void subscribeToLinkState(std::function<void(drone_sdk::safetyState)> callback)
    {
        m_safetySM.subscribeToLinkState(callback);
    }

private:
    flightstatemachine::FlightStateMachine m_flightSM;
    safetystatemachine::SafetyStateMachine m_safetySM;
    commandstatemachine::CommandStateMachine m_commandSM;
};

#endif // STATE_MACHINE_MANAGER_HPP
