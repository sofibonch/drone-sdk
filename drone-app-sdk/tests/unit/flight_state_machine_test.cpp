#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "flight_state_machine.hpp"
#include "icd.hpp"  // For SignalQuality and Location

using namespace flightstatemachine;
using namespace drone_sdk;

// Mock subscriber to capture state changes
class MockSubscriber {
public:
    MOCK_METHOD(void, onStateChanged, (FlightState state), ());
};

// Test fixture for FlightStateMachine
class FlightStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        flightSM = std::make_shared<FlightStateMachine>();
    }

    std::shared_ptr<FlightStateMachine> flightSM;
    MockSubscriber mockSubscriber;
};

// Test the transition from Landed -> Takeoff
TEST_F(FlightStateMachineTest, TestTransitionToTakeoff) {
    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::TAKEOFF)).Times(1);

    flightSM->subscribeToStateChange([this](FlightState state) {
        mockSubscriber.onStateChanged(state);
    });

    flightSM->triggerTakeoff();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::TAKEOFF);
}

// Test the transition from Takeoff -> Airborne
TEST_F(FlightStateMachineTest, TestTransitionToAirborne) {
    flightSM->triggerTakeoff();  // First transition to Takeoff
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::TAKEOFF);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::AIRBORNE)).Times(1);
    flightSM->triggerAirborne();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::AIRBORNE);
}

// Test the transition from Airborne -> Hover
TEST_F(FlightStateMachineTest, TestTransitionToHover) {
    flightSM->triggerAirborne();  // First transition to Airborne
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::AIRBORNE);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::HOVER)).Times(1);
    flightSM->triggerHover();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::HOVER);
}

// Test the transition from Hover -> Airborne
TEST_F(FlightStateMachineTest, TestTransitionBackToAirborneFromHover) {
    flightSM->triggerHover();  // First transition to Hover
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::HOVER);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::AIRBORNE)).Times(1);
    flightSM->triggerAirborne();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::AIRBORNE);
}

// Test the transition from Airborne -> ReturnHome (Task complete)
TEST_F(FlightStateMachineTest, TestTransitionToReturnHome) {
    flightSM->triggerAirborne();  // First transition to Airborne
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::AIRBORNE);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::RETURN_HOME)).Times(1);
    flightSM->triggerTaskComplete();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::RETURN_HOME);
}

// Test the transition from any state -> EmergencyLand (Safety violation)
TEST_F(FlightStateMachineTest, TestTransitionToEmergencyLand) {
    // Test for multiple states triggering an emergency landing
    flightSM->triggerTakeoff();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::TAKEOFF);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::EMERGENCY_LAND)).Times(1);
    flightSM->triggerSafetyViolation();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::EMERGENCY_LAND);

    flightSM->triggerAirborne();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::AIRBORNE);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::EMERGENCY_LAND)).Times(1);
    flightSM->triggerSafetyViolation();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::EMERGENCY_LAND);
}

// Test the transition from EmergencyLand -> Landed (Task complete)
TEST_F(FlightStateMachineTest, TestTransitionFromEmergencyLandToLanded) {
    flightSM->triggerSafetyViolation();  // First transition to EmergencyLand
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::EMERGENCY_LAND);

    EXPECT_CALL(mockSubscriber, onStateChanged(FlightState::LANDED)).Times(1);
    flightSM->triggerTaskComplete();
    EXPECT_EQ(flightSM->getCurrentState(), FlightState::LANDED);
}
