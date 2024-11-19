#include <iostream>
#include "flight_controller_handler.hpp"  // Include the FlightControllerHandler class header

void demoFlightControllerHandler() {
    FlightControllerHandler flightControllerHandler;

    std::cout << "Arming the flight controller..." << std::endl;
    auto status = flightControllerHandler.arm();
    if (status == drone_sdk::FlightControllerStatus::SUCCESS) {
        std::cout << "Flight controller armed successfully!" << std::endl;
    } else {
        std::cout << "Failed to arm the flight controller!" << std::endl;
        return;
    }

    std::cout << "Taking off..." << std::endl;
    drone_sdk::Location loc {2,3,4};
    status = flightControllerHandler.takeOff(loc);
    if (status == drone_sdk::FlightControllerStatus::SUCCESS) {
        std::cout << "Drone is airborne!" << std::endl;
    } else {
        std::cout << "Failed to take off!" << std::endl;
        return;
    }

    // Simulate some flying or task performing here (e.g., hover, navigate to a point)
    std::cout << "Performing a task..." << std::endl;

    std::cout << "Landing..." << std::endl;
    flightControllerHandler.land();
    
}

int main() {
    demoFlightControllerHandler();
    return 0;
}
