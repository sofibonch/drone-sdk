#ifndef COMMAND_CONTROLLER_HPP
#define COMMAND_CONTROLLER_HPP

#include "icd.hpp"
#include <queue>
#include <functional> // For std::function

// Include the real or mock flight controller handler based on DEBUG flag
#ifdef DEBUG
#include "mock_flight_controller_handler.hpp" // Mock implementation
#else
#include "flight_controller_handler.hpp" // Real implementation
#endif

class CommandController
{
public:
    CommandController() = default;
    ~CommandController() = default;

    void start(drone_sdk::Location home);
    drone_sdk::FlightControllerStatus goTo(const drone_sdk::Location &location);
    drone_sdk::FlightControllerStatus abortMission();
    drone_sdk::FlightControllerStatus hover();
    drone_sdk::FlightControllerStatus path(drone_sdk::Location firstPoint); // rest will get straight from the sm command machine
    void handleDestinationChange(drone_sdk::Location newDestination);
    void handleCommandState(drone_sdk::CommandStatus commandState);
    drone_sdk::FlightControllerStatus takingOff(drone_sdk::Location location);

    void setHome(drone_sdk::Location newHome)
    {
        m_homebase = newHome;
    }
    void updateCurrentLocation(const drone_sdk::Location &location)
    {
        m_currentLocation = location;
    }

private:
    // Use the FlightControllerHandler type which will resolve to either the real or mock handler at compile time
    #ifdef DEBUG
    MockFlightControllerHandler m_flightControllerHandler; // Mock version
    #else
    FlightControllerHandler m_flightControllerHandler; // Real version
    #endif
    drone_sdk::FlightState m_cur_flight_state;
    drone_sdk::Location m_homebase;
    bool m_onPath;
    bool m_onLand;
    drone_sdk::Location m_currentLocation;

    // Callbacks
    void onCommandStateChanged(drone_sdk::CommandStatus commandState);
};

#endif // COMMAND_CONTROLLER_HPP
