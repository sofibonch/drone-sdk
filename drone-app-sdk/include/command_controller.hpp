#ifndef COMMAND_CONTROLLER_HPP
#define COMMAND_CONTROLLER_HPP

#include "flight_controller_handler.hpp" // For FlightControllerHandler
#include "icd.hpp"
#include <queue>
#include <functional> // For std::function

class CommandController
{
public:
    CommandController() = default;
    ~CommandController() = default;

    void start(drone_sdk::Location home);
    drone_sdk::FlightControllerStatus goTo(const drone_sdk::Location &location);
    drone_sdk::FlightControllerStatus abortMission();
    drone_sdk::FlightControllerStatus hover();
    drone_sdk::FlightControllerStatus path(drone_sdk::Location firstPoint);
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
    // using PathProgressionSignal = boost::signals2::signal<void(drone_sdk::FlightControllerStatus)>;
private:
    FlightControllerHandler m_flightControllerHandler;

    drone_sdk::FlightState m_cur_flight_state;
    drone_sdk::Location m_homebase;
    bool m_onPath;
    bool m_onLand;
    drone_sdk::Location m_currentLocation;
    // Callbacks
    void onCommandStateChanged(drone_sdk::CommandStatus commandState);
};

#endif // COMMAND_CONTROLLER_HPP
