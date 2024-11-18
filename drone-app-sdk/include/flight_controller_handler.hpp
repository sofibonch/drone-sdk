#ifndef FLIGHT_CONTROLLER_HANDLER_HPP
#define FLIGHT_CONTROLLER_HANDLER_HPP

#include "flight-controller/flight_controller.hpp" 
#include "icd.hpp"

class FlightControllerHandler {
public:
    FlightControllerHandler() = default;

    drone_sdk::FlightControllerStatus arm()  {
        return convertResponse(m_flightController.arm());
    }

    drone_sdk::FlightControllerStatus disarm()  {
        return convertResponse(m_flightController.disarm());
    }

    drone_sdk::FlightControllerStatus takeOff(const drone_sdk::Location &location)  {
        return convertResponse(m_flightController.takeOff(location.altitude));
    }

    void land()  {
        m_flightController.land();
    }

    drone_sdk::FlightControllerStatus goHome()  {
        return convertResponse(m_flightController.goHome());
    }

    drone_sdk::FlightControllerStatus goTo(drone_sdk::Location location)  {
        return convertResponse(m_flightController.goTo(location.altitude, location.longitude, location.altitude));
    }

private:
    hw_sdk_mock::FlightController m_flightController;  // Real flight controller instance

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

#endif // FLIGHT_CONTROLLER_HANDLER_HPP
