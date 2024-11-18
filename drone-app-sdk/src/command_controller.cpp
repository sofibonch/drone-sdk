#include "command_controller.hpp"
#include "icd.hpp"

void CommandController::start(drone_sdk::Location home)
{
    m_homebase = home;
    m_onPath = false;
    m_onLand = true;
}

drone_sdk::FlightControllerStatus CommandController::hover(const drone_sdk::Location &cur_location)
{
    // Command the flight controller to hover
    // Hover at the current location
    return m_flightControllerHandler.goTo(cur_location);
}

drone_sdk::FlightControllerStatus CommandController::abortMission()
{
    // Abort mission by notifying the state machine and flight controller
    return m_flightControllerHandler.goHome();
}

drone_sdk::FlightControllerStatus CommandController::goTo(const drone_sdk::Location &newLocation)
{
    if (m_onLand)
    {
        drone_sdk::FlightControllerStatus flightStatus = takingOff(newLocation);
        if (flightStatus != drone_sdk::FlightControllerStatus::SUCCESS)
        {
            return flightStatus;
        }
    }
    return m_flightControllerHandler.goTo(newLocation);
}

drone_sdk::FlightControllerStatus CommandController::path(drone_sdk::Location firstPoint)
{
    m_onPath = true;
    return m_flightControllerHandler.goTo(firstPoint);
}
void CommandController::handleDestinationChange(drone_sdk::Location newDestination)
{
    m_flightControllerHandler.goTo(newDestination);
}

void CommandController::handleCommandState(drone_sdk::CommandStatus commandState)
{
    if (commandState == drone_sdk::CommandStatus::MISSION_ABORT)
    {
        m_flightControllerHandler.land();
    }
    // finished doing the path
    if (commandState == drone_sdk::CommandStatus::IDLE && m_onPath)
    {
        m_onPath = false;
    }
}

drone_sdk::FlightControllerStatus CommandController::takingOff(drone_sdk::Location location)
{
    drone_sdk::FlightControllerStatus flightStatus = m_flightControllerHandler.arm();

    if (flightStatus == drone_sdk::FlightControllerStatus::SUCCESS)
    {
        return m_flightControllerHandler.takeOff(location);
    }
    else
    {
        return flightStatus;
    }
}

void CommandController::onCommandStateChanged(drone_sdk::CommandStatus commandState)
{
    // Handle any additional logic needed when the command state changes
    handleCommandState(commandState);
}
