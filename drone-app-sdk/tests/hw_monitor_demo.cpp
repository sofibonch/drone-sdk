// hw_monitor_demo.cpp
#include <iostream>
#include "gps_handler.hpp"
#include "hw_monitor.hpp"

// Callback function for handling GPS updates
void gpsUpdateCallback(hw_sdk_mock::Gps::Location location, hw_sdk_mock::Gps::SignalQuality signalQuality) {
    std::cout << "Received GPS update: Location(" 
              << location.latitude << ", " << location.longitude << ", " << location.altitude 
              << "), Signal Quality: " << static_cast<int>(signalQuality) << std::endl;
}


int main() {
    // Initialize GpsHandler
    GpsHandler gpsHandler;

    // Subscribe to GPS updates
    gpsHandler.subscribe(gpsUpdateCallback);

    // Initialize HardwareMonitor with GpsHandler
    HardwareMonitor hwMonitor(gpsHandler);

    // Start polling at 10 Hz
    hwMonitor.start();

    // Run the demo for a limited time
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Stop polling and end the demo
    hwMonitor.stop();

    return 0;
}
