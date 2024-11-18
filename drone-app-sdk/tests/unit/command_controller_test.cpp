#include "command_controller.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "icd.hpp"
// Mock class for FlightControllerHandler
class MockFlightControllerHandler {
public:
    MOCK_METHOD(drone_sdk::FlightControllerStatus, arm, (), ());
    MOCK_METHOD(drone_sdk::FlightControllerStatus, takeOff, (double altitude), ());
    MOCK_METHOD(drone_sdk::FlightControllerStatus, goTo, (const drone_sdk::Location &location), ());
    MOCK_METHOD(drone_sdk::FlightControllerStatus, land, (), ());
    MOCK_METHOD(drone_sdk::FlightControllerStatus, goHome, (), ());
};

// Fixture class for CommandController tests
class CommandControllerTest : public ::testing::Test {
protected:
    MockFlightControllerHandler mockFlightControllerHandler;
    CommandController commandController;

    void SetUp() override {
        // Set up initial test environment
        drone_sdk::Location home{0.0, 0.0, 0.0};
        commandController.setHome(home);
    }
};


// Test goTo function when the drone is on land
TEST_F(CommandControllerTest, GoToTakesOffWhenLanded) {
    // Set expectations for takeoff and goTo
    EXPECT_CALL(mockFlightControllerHandler, arm())
        .WillOnce(::testing::Return(drone_sdk::FlightControllerStatus::SUCCESS));
    EXPECT_CALL(mockFlightControllerHandler, takeOff(::testing::_))
        .WillOnce(::testing::Return(drone_sdk::FlightControllerStatus::SUCCESS));
    EXPECT_CALL(mockFlightControllerHandler, goTo(::testing::_))
        .WillOnce(::testing::Return(drone_sdk::FlightControllerStatus::SUCCESS));

    drone_sdk::Location destination{10.0, 10.0, 10.0};
    auto status = commandController.goTo(destination);

    EXPECT_EQ(status, drone_sdk::FlightControllerStatus::SUCCESS);
}

// Test abortMission function
TEST_F(CommandControllerTest, AbortMissionCallsGoHome) {
    EXPECT_CALL(mockFlightControllerHandler, goHome())
        .WillOnce(::testing::Return(drone_sdk::FlightControllerStatus::SUCCESS));

    auto status = commandController.abortMission();

    EXPECT_EQ(status, drone_sdk::FlightControllerStatus::SUCCESS);
}

// Test path function
TEST_F(CommandControllerTest, PathSuccess) {
    EXPECT_CALL(mockFlightControllerHandler, goTo(::testing::_))
        .WillOnce(::testing::Return(drone_sdk::FlightControllerStatus::SUCCESS));

    drone_sdk::Location point{15.0, 15.0, 15.0};
    auto status = commandController.path(point);

    EXPECT_EQ(status, drone_sdk::FlightControllerStatus::SUCCESS);
}
