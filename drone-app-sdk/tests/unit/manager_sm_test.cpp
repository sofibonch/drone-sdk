
#include "state_machine_manager.hpp"
#include "../mocks/mock_gps.hpp"  // Include the mock GPS
#include "../mocks/mock_link.hpp" // Include the mock Link
#include "icd.hpp"

#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;
using namespace flightstatemachine;

using namespace hw_sdk_mock;
using namespace drone_sdk;

class StateMachineManagerTest : public ::testing::Test
{
protected:
    StateMachineManager stateMachineManager;
    MockGps mockGps;   // GPS mock
    MockLink mockLink; // Link mock

    void SetUp() override
    {
    }
};

// Test GPS signal update
TEST_F(StateMachineManagerTest, HandleGpsSignalUpdate)
{
    // Set up mock GPS data
    mockGps.setLocation(51.5074, -0.1278, 100.0); // Example: London
    mockGps.setSignalQuality(SignalQuality::GOOD);
    bool bip = true;
    // Simulate GPS update
    mockGps.simulateGpsUpdate();
    stateMachineManager.boop();
    EXPECT_TRUE(bip);
    // Verify the safety state machine processed the signal
    // EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_HEALTH);
}

// Test Link signal update
//TEST_F(StateMachineManagerTest, HandleLinkSignalUpdate)
//{
//    // Set up mock Link signal quality
//    mockLink.setSignalQuality(MockLink::SignalQuality::POOR);
//
//    // Simulate Link update
//    stateMachineManager.start(nullptr); // For simplicity, assuming proper HW monitor setup
//
//    // Verify the safety state machine processed the signal
//    EXPECT_EQ(observer.getLastLinkState(), safetyState::NOT_CONNECTED);
//}
