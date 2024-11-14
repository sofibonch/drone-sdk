#include <iostream>
#include <thread>
#include <chrono>
#include "gps_handler.hpp"

// Callback function that will be called when the GPS update signal is emitted
void gpsUpdateCallback(const drone_sdk::Location& location, drone_sdk::SignalQuality signalQuality) {
    std::cout << "Received GPS update:" << std::endl;
    std::cout << "Latitude: " << location.latitude << ", "
              << "Longitude: " << location.longitude << ", "
              << "Altitude: " << location.altitude << std::endl;

    std::cout << "Signal Quality: ";
    switch (signalQuality) {
        case drone_sdk::SignalQuality::NO_SIGNAL: std::cout << "No Signal"; break;
        case drone_sdk::SignalQuality::POOR: std::cout << "Poor"; break;
        case drone_sdk::SignalQuality::FAIR: std::cout << "Fair"; break;
        case drone_sdk::SignalQuality::GOOD: std::cout << "Good"; break;
        case drone_sdk::SignalQuality::EXCELLENT: std::cout << "Excellent"; break;
    }
    std::cout << std::endl;
}

int main() {
    // Instantiate GpsHandler
    GpsHandler gpsHandler;

    // Subscribe to GPS updates (callback function expects drone_sdk types)
    auto connection = gpsHandler.subscribe(&gpsUpdateCallback);

    // Simulate GPS updates periodically
    for (int i = 0; i < 5; ++i) {
        std::cout << "\nUpdating GPS data (" << i+1 << "/5)..." << std::endl;
        gpsHandler.update();  // Retrieve and publish GPS data
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Delay to simulate periodic updates
    }

    // Disconnect the signal when done
    connection.disconnect();

    return 0;
}
