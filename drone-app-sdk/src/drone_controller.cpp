#include "drone_controller.hpp"
#include <iostream>

DroneController::DroneController()
    : m_stateMachineManager(), m_commandController()
{
    m_hwMonitor.start();                       // Start monitoring
    m_stateMachineManager.start(&m_hwMonitor); // Start state machine
    m_commandController.start(m_stateMachineManager.getHome());
    m_hwMonitor.subscribeToGpsUpdates([this](const drone_sdk::Location &location, const drone_sdk::SignalQuality &signalQuality)
                                      { 
                                          m_commandController.updateCurrentLocation(location);
                                          (void)signalQuality;  // Cast to void to mark it as unused
                                      });
}


DroneController::~DroneController()
{
    m_hwMonitor.stop(); // Clean up resources
}

drone_sdk::FlightControllerStatus DroneController::goTo(const drone_sdk::Location &location)
{
    drone_sdk::FlightControllerStatus machineStat = drone_sdk::FlightControllerStatus::UNKNOWN_ERROR;
    try
    {
        // Attempt to assign a new task in the state machine manager
        machineStat = m_stateMachineManager.newTask(
            drone_sdk::CurrentMission::GOTO,
            location,    // Single destination passed as an optional parameter
            std::nullopt // pathDestinations is not used, so pass std::nullopt
        );

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the new task fails, throw an exception to trigger rollback
            throw std::runtime_error("StateMachineManager newTask failed");
        }

        // If the state machine task was successful, proceed with the command controller
        machineStat = m_commandController.goTo(location);

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the command controller fails, throw an exception to trigger rollback
            throw std::runtime_error("CommandController goTo failed");
        }

        // If both steps were successful, return success
        return drone_sdk::FlightControllerStatus::SUCCESS;
    }
    catch (const std::exception &e)
    {
        // Catch the exception and handle the rollback
        std::cerr << "Error: " << e.what() << std::endl;

        // If a failure occurred, return the state obtained from the command controller
        return machineStat; // Return the state from the `goTo` attempt in the CommandController
    }
}

// bool DroneController::followPath(const std::vector<drone_sdk::Location> &path){
//     return m_commandController.followPath(path);
// }
drone_sdk::FlightControllerStatus DroneController::abortMission()
{
    drone_sdk::FlightControllerStatus machineStat = drone_sdk::FlightControllerStatus::UNKNOWN_ERROR;
    try
    {
        // Attempt to assign an abort mission task in the state machine manager
        machineStat = m_stateMachineManager.newTask(
            drone_sdk::CurrentMission::EMERGENCY,
            drone_sdk::Location{}, // Empty location for abort mission
            std::nullopt           // pathDestinations is not used, so pass std::nullopt
        );

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the new task fails, throw an exception to trigger rollback
            throw std::runtime_error("StateMachineManager newTask failed for abort mission");
        }

        // If the state machine task was successful, proceed with the command controller
        machineStat = m_commandController.abortMission();

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the command controller fails, throw an exception to trigger rollback
            throw std::runtime_error("CommandController abortMission failed");
        }

        // If both steps were successful, return success
        return drone_sdk::FlightControllerStatus::SUCCESS;
    }
    catch (const std::exception &e)
    {
        // Catch the exception and handle the rollback
        std::cerr << "Error: " << e.what() << std::endl;

        // If a failure occurred, return the state obtained from the command controller
        return machineStat; // Return the state from the `abortMission` attempt in the CommandController
    }
}

drone_sdk::FlightControllerStatus DroneController::hover()
{
    drone_sdk::FlightControllerStatus machineStat = drone_sdk::FlightControllerStatus::UNKNOWN_ERROR;
    try
    {
        // Attempt to assign a hover task in the state machine manager
        machineStat = m_stateMachineManager.newTask(
            drone_sdk::CurrentMission::HOVER,
            drone_sdk::Location{}, // Empty location for hover
            std::nullopt           // pathDestinations is not used, so pass std::nullopt
        );

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the new task fails, throw an exception to trigger rollback
            throw std::runtime_error("StateMachineManager newTask failed for hover");
        }

        // If the state machine task was successful, proceed with the command controller
        machineStat = m_commandController.hover();

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the command controller fails, throw an exception to trigger rollback
            throw std::runtime_error("CommandController hover failed");
        }

        // If both steps were successful, return success
        return drone_sdk::FlightControllerStatus::SUCCESS;
    }
    catch (const std::exception &e)
    {
        // Catch the exception and handle the rollback
        std::cerr << "Error: " << e.what() << std::endl;

        // If a failure occurred, return the state obtained from the command controller
        return machineStat; // Return the state from the `hover` attempt in the CommandController
    }
}
