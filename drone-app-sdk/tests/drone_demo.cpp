#include "drone_sdk.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Create the DroneSDK instance
    DroneSDK drone;

    std::cout << "Starting Drone SDK...\n";

    // Execute the emergency land command
    drone.emergencyLand();
    drone.returnHome();
    // Optionally, you can also execute other commands like hover, returnHome, etc.
    // For example: drone.returnHome();

    // Sleep for a few seconds to simulate the SDK running and to allow the commands to take effect
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // The SDK is now stopping - you can call stop() on the DroneSDK to clean up
    std::cout << "Stopping Drone SDK...\n";

    // As the DroneSDK manages its controller lifecycle internally,
    // we don't need to manually stop anything here as it's done automatically.

    return 0;
}
