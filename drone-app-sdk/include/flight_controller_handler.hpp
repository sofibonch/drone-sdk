#ifndef FLIGHT_CONTROLLER_HANDLER_HPP
#define FLIGHT_CONTROLLER_HANDLER_HPP

#include "flight-controller/flight_controller.hpp"    // For hw_sdk_mock::FlightController
#include "icd.hpp"                  // For drone_sdk::FlightControllerStatus

class FlightControllerHandler {
public:
    FlightControllerHandler() = default;

    // Command to arm the flight controller
    drone_sdk::FlightControllerStatus arm() {
        return convertResponse(m_flightController.arm());
    }

    // Command to disarm the flight controller
    drone_sdk::FlightControllerStatus disarm() {
        return convertResponse(m_flightController.disarm());
    }

    // Command to take off
    drone_sdk::FlightControllerStatus takeOff(const drone_sdk::Location & location) {
        return convertResponse(m_flightController.takeOff(location.altitude));
    }

    // Command to land
    void land() {
        m_flightController.land();
    }

    // Command to return to home location
    drone_sdk::FlightControllerStatus goHome() {
        return convertResponse(m_flightController.goHome());
    }

    // Command to go to a specific location
    drone_sdk::FlightControllerStatus goTo(drone_sdk::Location location) {
        return convertResponse(m_flightController.goTo(location.altitude, location.longitude, location.altitude));
    }

private:
    hw_sdk_mock::FlightController m_flightController; // Directly instantiated

    // Helper function to convert hw_sdk_mock::FlightController::ResponseCode to drone_sdk::FlightControllerStatus
    drone_sdk::FlightControllerStatus convertResponse(hw_sdk_mock::FlightController::ResponseCode response) const {
        switch (response) {
            case hw_sdk_mock::FlightController::ResponseCode::SUCCESS:
                return drone_sdk::FlightControllerStatus::SUCCESS;
            case hw_sdk_mock::FlightController::ResponseCode::CONNECTION_ERROR:
                return drone_sdk::FlightControllerStatus::CONNECTION_ERROR;
            case hw_sdk_mock::FlightController::ResponseCode::HARDWARE_ERROR:
                return drone_sdk::FlightControllerStatus::HARDWARE_ERROR;
            case hw_sdk_mock::FlightController::ResponseCode::INVALID_COMMAND:
                return drone_sdk::FlightControllerStatus::INVALID_COMMAND;
            default:
                return drone_sdk::FlightControllerStatus::UNKNOWN_ERROR;
        }
    }
};

#endif