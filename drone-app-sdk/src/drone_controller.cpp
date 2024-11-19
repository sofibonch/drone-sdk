#include "drone_controller.hpp"
#include <iostream>

DroneController::DroneController()
    : m_stateMachineManager(), m_commandController()
{
    m_hwMonitor.start();
    m_stateMachineManager.start();
    m_commandController.start(m_stateMachineManager.getHome());
    m_hwMonitor.subscribeToGpsUpdates([this](const drone_sdk::Location &location, const drone_sdk::SignalQuality &signalQuality)
                                      {
                                          m_stateMachineManager.handleGpsUpdate(location, signalQuality);
                                          m_commandController.updateCurrentLocation(location); });
    m_hwMonitor.subscribeToLinkUpdates([this](const drone_sdk::SignalQuality &signalQuality)
                                       { m_stateMachineManager.handleLinkUpdate(signalQuality); });
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

drone_sdk::FlightControllerStatus DroneController::path(std::queue<drone_sdk::Location> path)
{

    drone_sdk::FlightControllerStatus machineStat = drone_sdk::FlightControllerStatus::UNKNOWN_ERROR;
    try
    {
        // Attempt to assign a new task in the state machine manager
        machineStat = m_stateMachineManager.newTask(
            drone_sdk::CurrentMission::PATH,
            std::nullopt,
            path);

        if (machineStat != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            // If the new task fails, throw an exception to trigger rollback
            throw std::runtime_error("StateMachineManager newTask failed");
        }

        // If the state machine task was successful, proceed with the command controller
        machineStat = m_commandController.goTo(path.front());

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

void DroneController::subscribeToGpsSignalState(std::function<void(drone_sdk::safetyState)> callback)
{
    m_stateMachineManager.subscribeToGpsSignalState(callback);
}
void DroneController::subscribeToLinkSignalState(std::function<void(drone_sdk::safetyState)> callback)
{
    m_stateMachineManager.subscribeToLinkSignalState(callback);
}

void DroneController::subscribeToGpsLocation(std::function<void(const drone_sdk::Location &, const drone_sdk::SignalQuality)> callback)
{
    m_stateMachineManager.subscribeToGpsUpdates(callback);
}

void DroneController::subscribeToFlightState(std::function<void(drone_sdk::FlightState)> callback)
{
    m_stateMachineManager.subscribeToFlightState(callback);
}

void DroneController::subscribeToCommandState(std::function<void(drone_sdk::CommandStatus)> callback)
{
    m_stateMachineManager.subscribeToCommandState(callback);
}
void DroneController::subscribeToWaypoint(std::function<void(drone_sdk::Location)> callback)
{
    m_stateMachineManager.subscribeToWaypoint(callback);
}

#ifdef DEBUG_MODE
#include <queue>

void DroneController::loadMockGpsData(const std::queue<drone_sdk::Location> &locations,
                                      const std::queue<drone_sdk::SignalQuality> &qualities)
{
    m_hwMonitor.loadMockGpsData(locations);
    m_hwMonitor.loadMockLinkData(qualities);
}

void DroneController::loadMockLinkData(const std::queue<drone_sdk::SignalQuality> &qualities)
{
    m_hwMonitor.loadMockLinkData(qualities);
}

void DroneController::runMockData()
{
    m_hwMonitor.start(); // Start processing the mock data
}
#endif
