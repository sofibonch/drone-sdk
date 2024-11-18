#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "command_controller.hpp"
#include "../mocks/sm_manager_mock.hpp" // Include the mock state machine
// #include "../mocks/hw_monitor_mock.hpp"           // Include the mock hardware monitor

using namespace drone_sdk;
using namespace testing;

// Test Fixture for CommandController
class CommandControllerTest : public ::testing::Test
{
protected:
    CommandController controller; // CommandController instance
    MockStateMachineManager mockStateMachine;
    // MockHwMonitor mockHwMonitor;

    Location homeBase = Location(0.0, 0.0, 0.0); // Default home base location

    void SetUp() override
    {
        // Set initial state for the controller
        controller.start(homeBase);
    }

};

// Test `goTo` method
TEST_F(CommandControllerTest, TestGoTo)
{
    Location destination(10.0, 20.0, 30.0);

    // Simulate behavior of goTo
    FlightControllerStatus status = controller.goTo(destination);

    EXPECT_EQ(status, FlightControllerStatus::SUCCESS); // Verify response
    // Additional checks for mock state machine interactions
}


// Test `goTo` method
TEST_F(CommandControllerTest, TestabbortMission)
{
    Location destination(10.0, 20.0, 30.0);

    // Simulate behavior of goTo
    FlightControllerStatus status = controller.abortMission();
    
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS); // Verify response
    // Additional checks for mock state machine interactions
}
