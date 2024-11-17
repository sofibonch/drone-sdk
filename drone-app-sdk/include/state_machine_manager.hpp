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

class StateMachineManager {
public:
    explicit StateMachineManager()=default;
    ~StateMachineManager()=default;

    void start(HardwareMonitor * hwMonitor){
        //subscribe  to signnals the safety 
        hwMonitor->subscribeToGpsUpdates([this](const drone_sdk::Location& location, drone_sdk::SignalQuality quality) {
            m_safetySM.handleGpsSignal(quality);
        });

        // Subscribe to Link updates
        hwMonitor->subscribeToLinkUpdates([this](drone_sdk::SignalQuality quality) {
            m_safetySM.handleLinkSignal(safetystatemachine::LinkSignal{quality});
        });


        // Subscribe to Link state changes from SafetyStateMachine and trigger emergency landing if disconnected
        //m_safetySM.subscribeToLinkState([this](drone_sdk::safetyState linkState) {
        //    m_flightSM.handleLinkStateChange(linkState);
        //});

        hwMonitor->subscribeToGpsUpdates([this](const drone_sdk::Location& location, drone_sdk::SignalQuality quality) {
            m_commandSM.handleGpsLocationUpdate(location);
        });

        hwMo
    }

   void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback) {
        m_flightSM.subscribeToStateChange(callback);
    }

   void subscribeToGpsSignalState(std::function<void(drone_sdk::FlightState)> callback) {
        m_safetySM.subscribeToGpsState(callback);
    }

    void subscribeToLinkSignalState(std::function<void(drone_sdk::FlightState)> callback) {
        m_safetySM.subscribeToLinkState(callback);
    }

    void subscribeToCommandState(std::function<void(drone_sdk::FlightState)> callback) {
        m_commandSM.subscribeToState(callback);
    }

    void subscribeToWaypoint(std::function<void(drone_sdk::FlightState)> callback) {
        m_commandSM.subscribeToPathWaypoint(callback);
    }


    // AssignTask function implementation using optional
    //void AssignTask(drone_sdk::CurrentMission mission,
    //                const std::optional<drone_sdk::Location>& singleDestination = std::nullopt,
    //                const std::optional<std::queue<drone_sdk::Location>>& pathDestinations = std::nullopt) {
    //    // Handle the task assignment with optional parameters
    //    m_commandSM.handleTaskAssigned(mission, singleDestination, pathDestinations);
    //}

    void SetHome(drone_sdk::Location newHome){
        m_commandSM.setHomebase(newHome);
    }
private:

    void subscribeToGpsState(std::function<void(drone_sdk::safetyState)> callback) {
        m_safetySM.subscribeToGpsState(callback);
    }

    void subscribeToLinkState(std::function<void(drone_sdk::safetyState)> callback) {
        m_safetySM.subscribeToLinkState(callback);
    }
    
private:
    flightstatemachine::FlightStateMachine m_flightSM;
    safetystatemachine::SafetyStateMachine m_safetySM;
    commandstatemachine::CommandStateMachine m_commandSM;
};


#endif // STATE_MACHINE_MANAGER_HPP
