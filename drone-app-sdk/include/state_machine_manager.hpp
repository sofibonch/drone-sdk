#ifndef STATE_MACHINE_MANAGER_HPP
#define STATE_MACHINE_MANAGER_HPP

#include "hw_monitor.hpp"
#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"
#include "icd.hpp"  // Include the ICD header for Location and SignalQuality
#include <boost/sml.hpp>
#include <iostream>
#include <memory>  // For std::shared_ptr

class StateMachineManager {
public:
    // Updated constructor to initialize SafetyStateMachine with a pointer to FlightStateMachine
    StateMachineManager(HardwareMonitor& hwMonitor)
        : m_flightSM(std::make_shared<flightstatemachine::FlightStateMachine>()), // Using shared_ptr to manage FlightStateMachine
          m_safetySM(m_flightSM) // Pass FlightStateMachine shared_ptr to SafetyStateMachine
    {
        // Subscribe to GPS updates using the ICD Location and SignalQuality
        hwMonitor.subscribeToGpsUpdates([this](const drone_sdk::Location& /*location*/, drone_sdk::SignalQuality quality) {
            // Trigger the safety state machine with the GPS signal
            std::cout << "Received GPS Signal Quality: " << static_cast<int>(quality) << std::endl;
            m_safetySM.handleGpsSignal(safetystatemachine::GpsSignal{quality});  // Trigger the safety state machine
        });

        // Subscribe to Link updates using the ICD SignalQuality
        hwMonitor.subscribeToLinkUpdates([this](drone_sdk::SignalQuality quality) {
            // Trigger the safety state machine with the Link signal
            std::cout << "Received Link Signal Quality: " << static_cast<int>(quality) << std::endl;
            m_safetySM.handleLinkSignal(safetystatemachine::LinkSignal{quality});  // Trigger the safety state machine
        });
    }

    void start() {
        std::cout << "State Machine Manager started!" << std::endl;
    }

private:
    std::shared_ptr<flightstatemachine::FlightStateMachine> m_flightSM;  // Flight state machine instance
    safetystatemachine::SafetyStateMachine m_safetySM;  // Safety state machine initialized with pointer to FlightStateMachine
};

#endif // STATE_MACHINE_MANAGER_HPP
