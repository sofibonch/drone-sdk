#ifndef DRONE_CONTROLLER_HPP
#define DRONE_CONTROLLER_HPP

#include "hw_monitor.hpp"            // For HardwareMonitor
#include "state_machine_manager.hpp" // For StateMachineManager
#include "command_controller.hpp"    // For CommandController
#include "icd.hpp"

class DroneController {
public:
    DroneController();
    ~DroneController();
 
bool goTo(const drone_sdk::Location & location);
//bool followPath(const std::vector<drone_sdk::Location> &path);
bool abortMission();
bool hover();

//HardwareMonitor& getHardwareMonitor() { return m_hwMonitor; }
private:
    HardwareMonitor m_hwMonitor;                // Monitors hardware components (e.g., GPS, Link)
    StateMachineManager m_stateMachineManager;  // Manages state transitions for the drone
    CommandController m_commandController;      // Manages commands
};

#endif // DRONE_CONTROLLER_HPP
