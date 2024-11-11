#include "drone_app_sdk.hpp"

namespace drone {

DroneAppSdk::DroneAppSdk() {
    std::cout << "DroneAppSdk Initialized." << std::endl;
}

DroneAppSdk::~DroneAppSdk() {
    std::cout << "DroneAppSdk Destroyed." << std::endl;
}

void DroneAppSdk::startMission() {
    std::cout << "Starting mission..." << std::endl;
    printStatus("Mission Started");
}

void DroneAppSdk::abortMission() {
    std::cout << "Mission aborted!" << std::endl;
    printStatus("Mission Aborted");
}

void DroneAppSdk::getStatus() {
    std::cout << "Fetching status..." << std::endl;
    printStatus("Mission in Progress");
}

void DroneAppSdk::printStatus(const std::string& status) {
    std::cout << "Status: " << status << std::endl;
}

} // namespace drone

int main() {
    drone::DroneAppSdk droneApp;

    droneApp.startMission();
    droneApp.getStatus();
    droneApp.abortMission();

    return 0;
}
