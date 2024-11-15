
#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"
#include "icd.hpp"

#include "../mocks/mock_gps.hpp"
#include "../mocks/mock_link.hpp"

#include <gtest/gtest.h>
#include <boost/signals2.hpp>

using namespace flightstatemachine;
using namespace hw_sdk_mock;

MockGps mockGps;
MockLink mockLink;
// Test transition to TAKEOFF state
TEST(FlightStateMachineTest, TransitionToTakeoff)
{
    FlightStateMachine fsm;

    fsm.triggerTakeoff();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::TAKEOFF);
}

// invalid Test transition to AIRBORNE state
TEST(FlightStateMachineTest, InvalidTransitionToAirborne)
{
    FlightStateMachine fsm;

    fsm.triggerAirborne();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);
}

//invalid Test transition to HOVER state
TEST(FlightStateMachineTest, InvalidTransitionToHover)
{
    FlightStateMachine fsm;

    fsm.triggerHover();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);
}

// Test transition to RETURN_HOME state after task completion
TEST(FlightStateMachineTest, IvalidTransitionToReturnHome)
{
    FlightStateMachine fsm;

    fsm.triggerTaskComplete();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);
}

// Test transition to EMERGENCY_LAND state due to safety violation
TEST(FlightStateMachineTest, TransitionToEmergencyLand)
{
    FlightStateMachine fsm;

    fsm.triggerSafetyViolation();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND);
}

// Test invalid transition from EMERGENCY_LAND to AIRBORNE
TEST(FlightStateMachineTest, InvalidTransitionFromEmergencyLandToAirborne)
{
    FlightStateMachine fsm;
    ;

    fsm.triggerTakeoff();
    fsm.triggerAirborne();
    fsm.triggerSafetyViolation(); // Transition to EMERGENCY_LAND

    // Try going airborne from EMERGENCY_LAND (invalid)
    fsm.triggerAirborne();
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND); // Should stay in EMERGENCY_LAND
}
