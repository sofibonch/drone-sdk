#include "flight-controller/flight_controller.hpp"
#include <iostream>
#include <random>
#include <string>

namespace hw_sdk_mock
{
    namespace
    {
        // Helper to generate random response codes
        FlightController::ResponseCode getRandomResponse()
        {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_int_distribution<int> dis(0, 3);
            return static_cast<FlightController::ResponseCode>(dis(gen));
        }

        // Function to convert ResponseCode to readable strings
        std::string responseCodeToString(FlightController::ResponseCode code)
        {
            switch (code)
            {
            case FlightController::ResponseCode::SUCCESS:
                return "SUCCESS";
            case FlightController::ResponseCode::CONNECTION_ERROR:
                return "CONNECTION_ERROR";
            case FlightController::ResponseCode::HARDWARE_ERROR:
                return "HARDWARE_ERROR";
            case FlightController::ResponseCode::INVALID_COMMAND:
                return "INVALID_COMMAND";
            default:
                return "UNKNOWN_RESPONSE";
            }
        }

        // Print the result
        void printResult(const std::string &command, FlightController::ResponseCode response)
        {
            std::cout << "Executing " << command << ": " << responseCodeToString(response) << "\n";
        }
    }

    FlightController::ResponseCode FlightController::arm()
    {
        ResponseCode response = getRandomResponse();
        printResult("ARM", response);
        return response;
    }

    FlightController::ResponseCode FlightController::disarm()
    {
        ResponseCode response = getRandomResponse();
        printResult("DISARM", response);
        return response;
    }

    FlightController::ResponseCode FlightController::takeOff()
    {
        ResponseCode response = getRandomResponse();
        printResult("TAKEOFF", response);
        return response;
    }

    FlightController::ResponseCode FlightController::land()
    {
        ResponseCode response = getRandomResponse();
        printResult("LAND", response);
        return response;
    }

    FlightController::ResponseCode FlightController::goHome()
    {
        ResponseCode response = getRandomResponse();
        printResult("GO_HOME", response);
        return response;
    }

    FlightController::ResponseCode FlightController::goTo(double latitude, double longitude, double altitude)
    {
        ResponseCode response = getRandomResponse();
        std::cout << "Executing GOTO (lat: " << latitude << ", lon: " << longitude << ", alt: " << altitude << "): "
                  << responseCodeToString(response) << "\n";
        return response;
    }

} // namespace hw_sdk_mock
