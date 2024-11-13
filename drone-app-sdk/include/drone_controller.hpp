#ifndef DRONE_CONTROLLER_HPP
#define DRONE_CONTROLLER_HPP

#include "hw_monitor.hpp"            // For HardwareMonitor
#include "state_machine_manager.hpp" // For StateMachineManager
#include "icd.hpp"

class DroneController {
public:
    DroneController();
    ~DroneController();

    // Command functions
    void hover();
    void emergencyLand();
    void returnHome();

    // Accessor for m_hwMonitor
    HardwareMonitor& getHardwareMonitor() { return m_hwMonitor; }

private:
    HardwareMonitor m_hwMonitor;                // Monitors hardware components (e.g., GPS, Link)
    StateMachineManager m_stateMachineManager;  // Manages state transitions for the drone
};

#endif // DRONE_CONTROLLER_HPP
