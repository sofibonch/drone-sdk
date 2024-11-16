#ifndef STATE_MACHINE_MANAGER_HPP
#define STATE_MACHINE_MANAGER_HPP

#include "hw_monitor.hpp"
#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"
#include "icd.hpp"
#include <boost/sml.hpp>
#include <iostream>
#include <memory>

class StateMachineManager {
public:
    StateMachineManager()
        : m_flightSM()//std::make_shared<flightstatemachine::FlightStateMachine>())  // FlightStateMachine managed by shared_ptr
        ,  m_safetySM()//m_flightSM)  // Pass FlightStateMachine pointer to SafetyStateMachine
    {
    }

    void start(HardwareMonitor * hwMonitor){
        hwMonitor->subscribeToGpsUpdates([this](const drone_sdk::Location& /*location*/, drone_sdk::SignalQuality quality) {
            m_safetySM.handleGpsSignal(safetystatemachine::GpsSignal{quality});
        });

        // Subscribe to Link updates
        hwMonitor->subscribeToLinkUpdates([this](drone_sdk::SignalQuality quality) {
            m_safetySM.handleLinkSignal(safetystatemachine::LinkSignal{quality});
        });
        
        // Subscribe to GPS state changes from SafetyStateMachine and trigger emergency landing if GPS is not healthy
        m_safetySM.subscribeToGpsState([this](drone_sdk::safetyState gpsState) {
            m_flightSM.handleGpsStateChange(gpsState);
        });

        // Subscribe to Link state changes from SafetyStateMachine and trigger emergency landing if disconnected
        m_safetySM.subscribeToLinkState([this](drone_sdk::safetyState linkState) {
            m_flightSM.handleLinkStateChange(linkState);
        });

    }

   void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback) {
        m_flightSM.subscribeToStateChange(callback);
    }

private:

    void subscribeToGpsState(std::function<void(drone_sdk::safetyState)> callback) {
        m_safetySM.subscribeToGpsState(callback);
    }

    void subscribeToLinkState(std::function<void(drone_sdk::safetyState)> callback) {
        m_safetySM.subscribeToLinkState(callback);
    }
    
private:
    flightstatemachine::FlightStateMachine m_flightSM;//std::shared_ptr<flightstatemachine::FlightStateMachine> m_flightSM;  // FlightStateMachine instance
    safetystatemachine::SafetyStateMachine m_safetySM;  // SafetyStateMachine with pointer to FlightStateMachine
};

#endif // STATE_MACHINE_MANAGER_HPP
