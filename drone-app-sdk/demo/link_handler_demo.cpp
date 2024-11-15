#include <iostream>
#include <thread>
#include <chrono>
#include "link_handler.hpp"

// Callback function for handling Link updates
void linkUpdateCallback(drone_sdk::SignalQuality signalQuality) {
    std::cout << "Received Link update: Signal Quality: " << static_cast<int>(signalQuality) << std::endl;
}

int main() {
    // Initialize LinkHandler
    LinkHandler linkHandler;

    // Subscribe to Link updates
    linkHandler.subscribe(linkUpdateCallback);

    // Simulate periodic updates
    for (int i = 0; i < 5; ++i) {
        // Update LinkHandler to simulate signal quality change
        linkHandler.update();

        // Sleep for a bit before the next update
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
