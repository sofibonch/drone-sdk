#pragma once

#include "flight_controller.hpp"  // Assuming this header is where hw_sdk_mock::FlightController is defined
#include <iostream>
#include <functional>

class FlightControllerHandler {
public:
    FlightControllerHandler()
        : m_flightController(std::make_unique<hw_sdk_mock::FlightController>()) {}

    // Command to arm the flight controller
    bool arm() {
        return executeCommand([this]() { return m_flightController->arm(); }, "Arm");
    }

    // Command to disarm the flight controller
    bool disarm() {
        return executeCommand([this]() { return m_flightController->disarm(); }, "Disarm");
    }

    // Command to take off
    bool takeOff() {
        return executeCommand([this]() { return m_flightController->takeOff(); }, "Take Off");
    }

    // Command to land
    bool land() {
        return executeCommand([this]() { return m_flightController->land(); }, "Land");
    }

    // Command to return to home location
    bool goHome() {
        return executeCommand([this]() { return m_flightController->goHome(); }, "Go Home");
    }

    // Command to go to a specific location
    bool goTo(double latitude, double longitude, double altitude) {
        return executeCommand([this, latitude, longitude, altitude]() {
            return m_flightController->goTo(latitude, longitude, altitude);
        }, "Go To Location");
    }

private:
    std::unique_ptr<hw_sdk_mock::FlightController> m_flightController;

    // Helper function to execute commands and handle response
    bool executeCommand(const std::function<hw_sdk_mock::FlightController::ResponseCode()>& command, const std::string& commandName) {
        hw_sdk_mock::FlightController::ResponseCode response = command();
        
        switch (response) {
            case hw_sdk_mock::FlightController::ResponseCode::SUCCESS:
                std::cout << commandName << " command succeeded.\n";
                return true;

            case hw_sdk_mock::FlightController::ResponseCode::CONNECTION_ERROR:
                std::cerr << commandName << " failed: Connection error.\n";
                break;

            case hw_sdk_mock::FlightController::ResponseCode::HARDWARE_ERROR:
                std::cerr << commandName << " failed: Hardware error.\n";
                break;

            case hw_sdk_mock::FlightController::ResponseCode::INVALID_COMMAND:
                std::cerr << commandName << " failed: Invalid command.\n";
                break;

            default:
                std::cerr << commandName << " failed: Unknown error.\n";
                break;
        }
        
        return false;
    }
};

