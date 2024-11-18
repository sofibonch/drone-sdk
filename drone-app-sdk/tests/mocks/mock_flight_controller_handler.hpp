#ifndef MOCK_FLIGHT_CONTROLLER_HANDLER_HPP
#define MOCK_FLIGHT_CONTROLLER_HANDLER_HPP

#include "icd.hpp" // For drone_sdk::FlightControllerStatus

class MockFlightControllerHandler
{
public:
    MockFlightControllerHandler() = default;

    // Simulated command to arm the flight controller
    drone_sdk::FlightControllerStatus arm()
    {
        // Mocked response
        return returnVal;
    }

    // Simulated command to disarm the flight controller
    drone_sdk::FlightControllerStatus disarm()
    {
        // Mocked response
        return returnVal;
    }

    // Simulated command to take off
    drone_sdk::FlightControllerStatus takeOff(const drone_sdk::Location &location)
    {
        // Mocked response
        (void)location;
        return returnVal;
    }

    // Simulated command to land
    void land()
    {
        // Mocked behavior
    }

    // Simulated command to return to home location
    drone_sdk::FlightControllerStatus goHome()
    {
        // Mocked response
        return returnVal;
    }

    // Simulated command to go to a specific location
    drone_sdk::FlightControllerStatus goTo(drone_sdk::Location location)
    {
        // Mocked response
        (void)location;
        return returnVal;
    }
    void setUpReturnVal(drone_sdk::FlightControllerStatus newVal)
    {
        returnVal = newVal;
    }

private:
    drone_sdk::FlightControllerStatus returnVal = drone_sdk::FlightControllerStatus::SUCCESS;
};

#endif // MOCK_FLIGHT_CONTROLLER_HANDLER_HPP
