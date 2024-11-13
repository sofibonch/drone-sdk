#include <iostream>
#include "hw_monitor.hpp"

// Callback for GPS updates
void gpsUpdateCallback(hw_sdk_mock::Gps::Location location, hw_sdk_mock::Gps::SignalQuality signalQuality) {
    std::cout << "Received GPS update: Location(" 
              << location.latitude << ", " << location.longitude << ", " << location.altitude 
              << "), Signal Quality: " << static_cast<int>(signalQuality) << std::endl;
}

// Callback for Link updates
void linkUpdateCallback(hw_sdk_mock::Link::SignalQuality signalQuality) {
    std::cout << "Received Link update: Signal Quality: " << static_cast<int>(signalQuality) << std::endl;
}

int main() {
    // Initialize HardwareMonitor (which contains both GPS and Link handlers)
    HardwareMonitor hwMonitor;

    // Subscribe to GPS and Link updates
    hwMonitor.subscribeToGpsUpdates(gpsUpdateCallback);
    hwMonitor.subscribeToLinkUpdates(linkUpdateCallback);

    // Start polling at 10 Hz
    hwMonitor.start();

    // Run the demo for a limited time (5 seconds)
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Stop polling and end the demo
    hwMonitor.stop();

    return 0;
}
