#ifndef STATE_MACHINE_MANAGER_HPP
#define STATE_MACHINE_MANAGER_HPP

#include "hw_monitor.hpp"
#include "state_machines/safety_state_machine.hpp"
#include "icd.hpp"  // Include the ICD header for Location and SignalQuality
#include <boost/sml.hpp>
#include <iostream>

class StateMachineManager {
public:
    StateMachineManager(HardwareMonitor& hwMonitor)
        : m_sm{}  // Initialize the safety state machine here
    {
        // Subscribe to GPS updates using the ICD Location and SignalQuality
        hwMonitor.subscribeToGpsUpdates([this](const drone_sdk::Location& /*location*/, drone_sdk::SignalQuality quality) {
            std::cout << "Received GPS Signal Quality: " << static_cast<int>(quality) << std::endl;
            m_sm.process_event(GpsSignal{quality});
        });

        // Subscribe to Link updates using the ICD SignalQuality
        hwMonitor.subscribeToLinkUpdates([this](drone_sdk::SignalQuality quality) {
            std::cout << "Received Link Signal Quality: " << static_cast<int>(quality) << std::endl;
            m_sm.process_event(LinkSignal{quality});
        });
    }

    void start() {
        std::cout << "State Machine Manager started!" << std::endl;
    }

private:
    boost::sml::sm<SafetyStateMachine> m_sm;  // Safety state machine instance
};

#endif // STATE_MACHINE_MANAGER_HPP
