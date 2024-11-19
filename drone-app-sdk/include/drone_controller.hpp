#ifndef DRONE_CONTROLLER_HPP
#define DRONE_CONTROLLER_HPP

#include "icd.hpp"
#include "command_controller.hpp"    // For CommandController
#include "state_machine_manager.hpp" // For StateMachineManager

#ifdef DEBUG_MODE
#include "mock_hw_monitor.hpp" // Use MockHwMonitor in debug mode
#else
#include "hw_monitor.hpp" // Use actual HardwareMonitor otherwise
#endif

#include <queue> // for path, should go to icd
#include <functional>

class DroneController
{
public:
    DroneController();
    ~DroneController();

    // Command actions
    drone_sdk::FlightControllerStatus goTo(const drone_sdk::Location &location);
    drone_sdk::FlightControllerStatus abortMission();
    drone_sdk::FlightControllerStatus hover();
    drone_sdk::FlightControllerStatus path(std::queue<drone_sdk::Location>);

    // Subscription functions
    void subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback);
    void subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback);
    void subscribeToGpsLocation(std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> callback);
    void subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback);
    void subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback);
    void subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback);

#ifdef DEBUG_MODE
    // Functions for loading and running mock data in debug mode
    void loadMockGpsData(const std::queue<drone_sdk::Location> &locations, const std::queue<drone_sdk::SignalQuality> &qualities);
    void loadMockLinkData(const std::queue<drone_sdk::SignalQuality> &qualities);
    void runMockData(); // Starts the mock monitor to process the loaded data
#endif
private:
#ifdef DEBUG_MODE
    MockHwMonitor m_hwMonitor; // Mock hardware monitor for debugging
#else
    HardwareMonitor m_hwMonitor; // Actual hardware monitor
#endif
    StateMachineManager m_stateMachineManager; // Manages state transitions for the drone
    CommandController m_commandController;     // Manages commands
};

#endif // DRONE_CONTROLLER_HPP
