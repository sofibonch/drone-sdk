#include "state_machines/flight_state_machine.hpp"
#include "icd.hpp"

#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;
using namespace flightstatemachine;

// Helper class to capture state changes
class FlightStateChangeObserver {
public:
    void onStateChanged(drone_sdk::FlightState newState) {
        lastState = newState;
    }

    drone_sdk::FlightState getLastState() const { return lastState; }

private:
    drone_sdk::FlightState lastState = drone_sdk::FlightState::LANDED;
};

class FlightStateMachineTest : public ::testing::Test {
protected:
    FlightStateMachine fsm;
    FlightStateChangeObserver observer;

    void SetUp() override {
        // Subscribe the observer to the state machine
        fsm.subscribeToStateChange(
            boost::bind(&FlightStateChangeObserver::onStateChanged, &observer, _1));
    }
};

// Test: Initial State
TEST_F(FlightStateMachineTest, InitialState) {
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);
}

// Section: Tests for triggerTakeoff()
TEST_F(FlightStateMachineTest, TriggerTakeoff) {
    fsm.triggerTakeoff();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::TAKEOFF);
    //EXPECT_TRUE(observer.wasNotified());
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::TAKEOFF);
}

// Section: Tests for triggerAirborne()
TEST_F(FlightStateMachineTest, TriggerAirborneValidTransition) {

    fsm.triggerTakeoff(); // Move to TAKEOFF first
    fsm.triggerAirborne();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::AIRBORNE);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::AIRBORNE);
}

TEST_F(FlightStateMachineTest, TriggerAirborneInvalidTransition) {
    fsm.triggerAirborne(); // Directly from LANDED (invalid)
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);

}

// Section: Tests for triggerHover()
TEST_F(FlightStateMachineTest, TriggerHoverValidTransition) {
    fsm.triggerTakeoff();
    fsm.triggerAirborne(); // Must be airborne first
    fsm.triggerHover();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::HOVER);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::HOVER);
}

TEST_F(FlightStateMachineTest, TriggerHoverInvalidTransition) {
    fsm.triggerHover(); // Directly from LANDED (invalid)
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);

}

// Section: Tests for triggerTaskComplete()
TEST_F(FlightStateMachineTest, TriggerTaskCompleteValidTransition) {
    fsm.triggerTakeoff();
    fsm.triggerAirborne(); // Must be airborne first
    fsm.triggerTaskComplete();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::RETURN_HOME);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::RETURN_HOME);
}

TEST_F(FlightStateMachineTest, TriggerTaskCompleteInvalidTransition) {
    fsm.triggerTaskComplete(); // Directly from LANDED (invalid)
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);

}

// Section: Tests for triggerSafetyViolation()
TEST_F(FlightStateMachineTest, TriggerSafetyViolation) {
    fsm.triggerTakeoff();
    fsm.triggerAirborne();
    fsm.triggerSafetyViolation();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::EMERGENCY_LAND);
}

// Section: Tests for handleGpsStateChange()
TEST_F(FlightStateMachineTest, HandleGpsStateChangeNotHealthy) {
    fsm.handleGpsStateChange(drone_sdk::safetyState::GPS_NOT_HEALTHY);
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::EMERGENCY_LAND);
}

// Section: Tests for handleLinkStateChange()
TEST_F(FlightStateMachineTest, HandleLinkStateChangeDisconnected) {
    fsm.handleLinkStateChange(drone_sdk::safetyState::NOT_CONNECTED);
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::EMERGENCY_LAND);
}

// Section: Invalid transitions from EMERGENCY_LAND
TEST_F(FlightStateMachineTest, InvalidTransitionFromEmergencyLand) {
    fsm.triggerSafetyViolation(); // Move to EMERGENCY_LAND
    fsm.triggerTakeoff();         // Invalid from EMERGENCY_LAND
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND);

}

TEST_F(FlightStateMachineTest, ReturnFromEmergencyLandToLanded) {
    fsm.triggerSafetyViolation(); // Move to EMERGENCY_LAND
    fsm.triggerTaskComplete();    // Task complete returns to LANDED
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);

    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::LANDED);
}
