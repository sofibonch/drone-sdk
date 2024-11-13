#include "drone_sdk.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Create the DroneSDK instance
    DroneSDK drone;

    std::cout << "Starting Drone SDK...\n";
    drone.emergencyLand();
    // Start the DroneSDK (initializes hardware monitoring and state machine)
   

    // Sleep for a few seconds to simulate the SDK running
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Stop the DroneSDK (clean up resources)
    std::cout << "Stopping Drone SDK...\n";
   // sdk.stop();

    return 0;
}
