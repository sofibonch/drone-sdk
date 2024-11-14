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
    StateMachineManager(HardwareMonitor& hwMonitor)
        : m_flightSM(std::make_shared<flightstatemachine::FlightStateMachine>()),  // FlightStateMachine managed by shared_ptr
          m_safetySM(m_flightSM)  // Pass FlightStateMachine pointer to SafetyStateMachine
    {
        // Subscribe to GPS updates
        hwMonitor.subscribeToGpsUpdates([this](const drone_sdk::Location& /*location*/, drone_sdk::SignalQuality quality) {
            m_safetySM.handleGpsSignal(safetystatemachine::GpsSignal{quality});
        });

        // Subscribe to Link updates
        hwMonitor.subscribeToLinkUpdates([this](drone_sdk::SignalQuality quality) {
            m_safetySM.handleLinkSignal(safetystatemachine::LinkSignal{quality});
        });

        // Subscribe to GPS state changes from SafetyStateMachine and trigger emergency landing if GPS is not healthy
        m_safetySM.subscribeToGpsState([this](drone_sdk::safetyState gpsState) {
            if (gpsState == drone_sdk::safetyState::GPS_NOT_HEALTHY) {
                std::cout << "GPS Not Healthy: Triggering Emergency Land in FlightStateMachine" << std::endl;
                m_flightSM->triggerSafetyViolation();  // Trigger emergency landing
            }
        });

        // Subscribe to Link state changes from SafetyStateMachine and trigger emergency landing if disconnected
        m_safetySM.subscribeToLinkState([this](drone_sdk::safetyState linkState) {
            if (linkState == drone_sdk::safetyState::NOT_CONNECTED) {
                std::cout << "Link Disconnected: Triggering Emergency Land in FlightStateMachine" << std::endl;
                m_flightSM->triggerSafetyViolation();  // Trigger emergency landing
            }
        });

        // Subscribe to state changes from FlightStateMachine
        m_flightSM->stateChanged.connect([this](const std::string& newState) {
            std::cout << "FlightStateMachine state changed to: " << newState << std::endl;
        });
    }

    void start() {
        std::cout << "State Machine Manager started!" << std::endl;
    }

private:
    std::shared_ptr<flightstatemachine::FlightStateMachine> m_flightSM;  // FlightStateMachine instance
    safetystatemachine::SafetyStateMachine m_safetySM;  // SafetyStateMachine with pointer to FlightStateMachine
};

#endif // STATE_MACHINE_MANAGER_HPP
