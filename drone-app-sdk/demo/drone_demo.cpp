#include "drone_sdk.hpp" // Include the DroneSDK header
#include <iostream>         // For standard I/O
#include <queue>            // For std::queue
#include <thread>           // For std::this_thread::sleep_for
#include <chrono>           // For std::chrono::seconds

int main()
{
    // Instantiate the DroneSDK
    DroneSDK drone;

    // Subscribe to GPS location updates
    drone.subscribeToGpsLocation([](const drone_sdk::Location &location, const drone_sdk::SignalQuality quality) {
        std::cout << "GPS Location Updated: "
                  << "Latitude: " << location.latitude
                  << ", Longitude: " << location.longitude
                  << ", Altitude: " << location.altitude
                  << " (Signal Quality: " << static_cast<int>(quality) << ")" << std::endl;
    });

    // Subscribe to flight state changes
    drone.subscribeToFlightState([](drone_sdk::FlightState state) {
        std::cout << "Flight State Changed: " << static_cast<int>(state) << std::endl;
    });

    // Subscribe to waypoint updates
    drone.subscribeToWaypoint([](drone_sdk::Location waypoint) {
        std::cout << "Reached Waypoint: "
                  << "Latitude: " << waypoint.latitude
                  << ", Longitude: " << waypoint.longitude
                  << ", Altitude: " << waypoint.altitude << std::endl;
    });

    // Simulate waiting to demonstrate subscription updates
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Example: Command the drone to go to a specific location
    drone_sdk::Location targetLocation = {1,1.5,1}; 
    auto status = drone.goTo(targetLocation);
    if (status == drone_sdk::FlightControllerStatus::SUCCESS)
    {
        std::cout << "Drone is heading to the target location." << std::endl;
    }
    else
    {
        std::cerr << "Failed to command the drone to go to the target location. Status: " << static_cast<int>(status) << std::endl;
    }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    // Example: Send the drone along a path of locations
    std::queue<drone_sdk::Location> path;
    path.push({1, 1,1}); 
    path.push({2,2,2}); 
    path.push({3,3,3});  

    status = drone.path(path);
    if (status == drone_sdk::FlightControllerStatus::SUCCESS)
    {
        std::cout << "Drone is following the path." << std::endl;
    }
    else
    {
        std::cerr << "Failed to command the drone to follow the path. Status: " << static_cast<int>(status) << std::endl;
    }

    // Example: Command the drone to hover
    status = drone.hover();
    if (status == drone_sdk::FlightControllerStatus::SUCCESS)
    {
        std::cout << "Drone is hovering." << std::endl;
    }
    else
    {
        std::cerr << "Failed to command the drone to hover. Status: " << static_cast<int>(status) << std::endl;
    }

    // Example: Abort the current mission
    status = drone.abortMission();
    if (status == drone_sdk::FlightControllerStatus::SUCCESS)
    {
        std::cout << "Mission aborted successfully." << std::endl;
    }
    else
    {
        std::cerr << "Failed to abort the mission. Status: " << static_cast<int>(status) << std::endl;
    }

    // Simulate waiting to demonstrate subscription updates
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Demo finished." << std::endl;
    return 0;
}
