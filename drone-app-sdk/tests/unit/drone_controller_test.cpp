#ifndef DRONE_CONTROLLER_TEST_HPP
#define DRONE_CONTROLLER_TEST_HPP

#include <gtest/gtest.h>
#include "drone_controller.hpp"
#include "mock_hw_monitor.hpp"

// TestObserver class to track the subscription callback calls
class TestObserver
{
public:
    void onGpsSignalStateUpdate(drone_sdk::safetyState state)
    {
        m_lastGpsSignalState = state;
        m_gpsSignalStateCalled = true;
    }

    void onLinkSignalStateUpdate(drone_sdk::safetyState state)
    {
        m_lastLinkSignalState = state;
        m_linkSignalStateCalled = true;
    }

    void onGpsLocationUpdate(const drone_sdk::Location &location, const drone_sdk::SignalQuality &quality)
    {
        m_lastLocation = location;
        m_lastSignalQuality = quality;
        m_gpsLocationCalled = true;
    }

    void onFlightStateUpdate(drone_sdk::FlightState state)
    {
        m_lastFlightState = state;
        m_flightStateCalled = true;
    }

    void onCommandStateUpdate(drone_sdk::CommandStatus status)
    {
        m_lastCommandStatus = status;
        m_commandStateCalled = true;
    }

    void onWaypointUpdate(const drone_sdk::Location &location)
    {
        m_lastWaypoint = location;
        m_waypointCalled = true;
    }

    // Reset flags for testing
    void reset()
    {
        m_gpsSignalStateCalled = false;
        m_linkSignalStateCalled = false;
        m_gpsLocationCalled = false;
        m_flightStateCalled = false;
        m_commandStateCalled = false;
        m_waypointCalled = false;
    }

    bool m_gpsSignalStateCalled = false;
    bool m_linkSignalStateCalled = false;
    bool m_gpsLocationCalled = false;
    bool m_flightStateCalled = false;
    bool m_commandStateCalled = false;
    bool m_waypointCalled = false;

    drone_sdk::safetyState m_lastGpsSignalState;
    drone_sdk::safetyState m_lastLinkSignalState;
    drone_sdk::Location m_lastLocation;
    drone_sdk::SignalQuality m_lastSignalQuality;
    drone_sdk::FlightState m_lastFlightState;
    drone_sdk::CommandStatus m_lastCommandStatus;
    drone_sdk::Location m_lastWaypoint;
};

// Test class for DroneController
class DroneControllerSelfLoadingTest : public ::testing::Test
{
protected:
    DroneController m_droneController;

    void SetUp() override
    {
    }

    void TearDown() override
    {
    }

    // Function to test if the subscriptions are correctly set up
    void InitSubscriptions(TestObserver &observer)
    {
        // Subscribe the observer functions to the DroneController signals
        m_droneController.subscribeToGpsSignalState([&observer](drone_sdk::safetyState state)
                                                    { observer.onGpsSignalStateUpdate(state); });

        m_droneController.subscribeToLinkSignalState([&observer](drone_sdk::safetyState state)
                                                     { observer.onLinkSignalStateUpdate(state); });

        m_droneController.subscribeToGpsLocation([&observer](const drone_sdk::Location &location, const drone_sdk::SignalQuality &quality)
                                                 { observer.onGpsLocationUpdate(location, quality); });

        m_droneController.subscribeToFlightState([&observer](drone_sdk::FlightState state)
                                                 { observer.onFlightStateUpdate(state); });

        m_droneController.subscribeToCommandState([&observer](drone_sdk::CommandStatus status)
                                                  { observer.onCommandStateUpdate(status); });

        m_droneController.subscribeToWaypoint([&observer](drone_sdk::Location location)
                                              { observer.onWaypointUpdate(location); });
    }
};

// Test case to verify that the safety state subscriptions work correctly
TEST_F(DroneControllerSelfLoadingTest, InitSubscriptionsTest)
{
    TestObserver observer;

    // Initialize subscriptions
    InitSubscriptions(observer);

    // Simulate updates to verify subscriptions
#ifdef DEBUG_MODE
    std::queue<drone_sdk::Location> locations;
    std::queue<drone_sdk::SignalQuality> gpsQualities;
    std::queue<drone_sdk::SignalQuality> linkQualities;

    locations.push({1, 2, 3});
    gpsQualities.push(drone_sdk::SignalQuality::EXCELLENT);
    linkQualities.push(drone_sdk::SignalQuality::EXCELLENT);

    locations.push({1, 2.5, 3});
    gpsQualities.push(drone_sdk::SignalQuality::EXCELLENT);
    linkQualities.push(drone_sdk::SignalQuality::NO_SIGNAL);

    m_droneController.loadMockGpsData(locations, gpsQualities);
    m_droneController.loadMockLinkData(linkQualities);
    m_droneController.runMockData();
    
//    std::this_thread::sleep_for(std::chrono::milliseconds(150)); // Wait for mock updates to propagate
#endif
    // Verify if observer callbacks were triggered
    std::this_thread::sleep_for(std::chrono::milliseconds(450)); // Wait for mock updates to propagate
    EXPECT_FALSE(observer.m_gpsSignalStateCalled);
    EXPECT_TRUE(observer.m_linkSignalStateCalled);
    EXPECT_TRUE(observer.m_gpsLocationCalled);

     EXPECT_EQ(observer.m_lastLocation.latitude, 1);
     EXPECT_EQ(observer.m_lastGpsSignalState, drone_sdk::safetyState::NOT_CONNECTED);
}

#endif // DRONE_CONTROLLER_TEST_HPP
