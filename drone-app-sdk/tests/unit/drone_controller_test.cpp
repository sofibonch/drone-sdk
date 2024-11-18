#include <gtest/gtest.h>
#include "drone_controller.hpp"
#include "../mocks/mock_hw_monitor.hpp"
#include "icd.hpp"

class DroneControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize the mock hardware monitor and drone controller
        m_mockHardwareMonitor = std::make_unique<MockHardwareMonitor>();
        m_droneController = std::make_unique<DroneController>();

        // Start the hardware monitor
        m_mockHardwareMonitor->start();
    }

    void TearDown() override {
        // Stop the hardware monitor after the test
        m_mockHardwareMonitor->stop();
    }

    std::unique_ptr<MockHardwareMonitor> m_mockHardwareMonitor;
    std::unique_ptr<DroneController> m_droneController;
};

// Test for the goTo command (moving vertically first, then horizontally)
TEST_F(DroneControllerTest, GoToCommandVerticalThenHorizontal) {
    // Simulate the GPS locations with increasing altitude first
    drone_sdk::Location targetLocation = {10.0, 10.0, 100.0}; // Target: xy=10,10, z=100 (altitude)
    
    // Create a mock path with the vertical progression first
    std::queue<drone_sdk::Location> mockPath;
    for (int z = 0; z <= 100; z += 10) {  // Increase altitude slowly in steps of 10
        mockPath.push({0.0, 0.0, static_cast<double>(z)});
    }

    // Now push the horizontal destination (xy values)
    mockPath.push(targetLocation);

    // Load the mock data into the hardware monitor
    m_mockHardwareMonitor->loadMockGpsData(mockPath);
    
    // Subscribe to GPS updates to handle location updates
    m_mockHardwareMonitor->subscribeToGpsUpdates([this](const drone_sdk::Location& location, const drone_sdk::SignalQuality&) {
        // You can perform checks on the location here if needed
        if (location.altitude == 100.0) {  // If the altitude is reached
            // Now test horizontal progression to target location
            EXPECT_EQ(location.latitude, 10.0);  // Use latitude instead of x
            EXPECT_EQ(location.longitude, 10.0);  // Use longitude instead of y
        }
    });

    // Execute the goTo command to start the movement
    auto status = m_droneController->goTo(targetLocation);
    
    // Check that the status is success (this may vary depending on your implementation)
    EXPECT_EQ(status, drone_sdk::FlightControllerStatus::SUCCESS);
}
