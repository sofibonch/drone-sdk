#include <iostream>
#include <thread>
#include <chrono>
#include "gps_handler.hpp"

// Callback function that will be called when the GPS update signal is emitted
void gpsUpdateCallback(const hw_sdk_mock::Gps::Location& location, hw_sdk_mock::Gps::SignalQuality signalQuality) {
    std::cout << "Received GPS update:" << std::endl;
    std::cout << "Latitude: " << location.latitude << ", "
              << "Longitude: " << location.longitude << ", "
              << "Altitude: " << location.altitude << std::endl;

    std::cout << "Signal Quality: ";
    switch (signalQuality) {
        case hw_sdk_mock::Gps::SignalQuality::NO_SIGNAL: std::cout << "No Signal"; break;
        case hw_sdk_mock::Gps::SignalQuality::POOR: std::cout << "Poor"; break;
        case hw_sdk_mock::Gps::SignalQuality::FAIR: std::cout << "Fair"; break;
        case hw_sdk_mock::Gps::SignalQuality::GOOD: std::cout << "Good"; break;
        case hw_sdk_mock::Gps::SignalQuality::EXCELLENT: std::cout << "Excellent"; break;
    }
    std::cout << std::endl;
}

int main() {
    // Instantiate GpsHandler
    GpsHandler gpsHandler;

    // Subscribe to GPS updates
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
