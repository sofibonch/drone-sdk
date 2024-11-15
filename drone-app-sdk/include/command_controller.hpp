#ifndef COMMAND_CONTROLLER_HPP
#define COMMAND_CONTROLLER_HPP

#include "flight_controller_handler.hpp"  // For FlightControllerHandler
#include "state_machine_manager.hpp"      // For StateMachineManager
#include "icd.hpp"
#include <queue>

class CommandController {
public:
    CommandController()=default;
    ~CommandController()=default;

    void start(StateMachineManager * stateMachineManager);
    bool goTo(const drone_sdk::Location& location);
    //bool followPath(const std::queue<drone_sdk::Location>& path);  // Updated to use std::queue
    bool abortMission();
    bool hover();

private:
    StateMachineManager* m_stateMachineManager;
    FlightControllerHandler m_flightControllerHandler;

    //void handleStatus(drone_sdk::FlightControllerStatus status);
    drone_sdk::FlightState m_cur_flight_state;
    drone_sdk::Location m_cur_location;
    drone_sdk::Location m_homebase;
    std::queue<drone_sdk::Location> m_pathQueue;  // Queue for path points
    bool m_onPath;
};

#endif // COMMAND_CONTROLLER_HPP
