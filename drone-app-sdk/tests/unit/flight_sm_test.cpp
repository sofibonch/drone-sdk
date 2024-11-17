#include "state_machines/flight_state_machine.hpp"
#include "icd.hpp"

#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;
using namespace flightstatemachine;

// Helper class to capture state changes
class FlightStateChangeObserver
{
public:
    void onStateChanged(drone_sdk::FlightState newState)
    {
        lastState = newState;
    }

    drone_sdk::FlightState getLastState() const { return lastState; }

private:
    drone_sdk::FlightState lastState = drone_sdk::FlightState::LANDED;
};

class FlightStateMachineTest : public ::testing::Test
{
protected:
    FlightStateMachine fsm;
    FlightStateChangeObserver observer;

    void SetUp() override
    {
        // Subscribe the observer to the state machine
        fsm.subscribeToStateChange(
            boost::bind(&FlightStateChangeObserver::onStateChanged, &observer, _1));
    }
};

// Test: Initial State
TEST_F(FlightStateMachineTest, InitialState)
{
    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::LANDED);
}


TEST_F(FlightStateMachineTest, HandleAirborneMission)
{
    // Simulate a mission that transitions to Airborne
    drone_sdk::CurrentMission airborneMission = drone_sdk::CurrentMission::GOTO; // Correct use of enum
    fsm.handleNewMission(airborneMission);     // Transition from Takeoff to Airborne

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::AIRBORNE);
}

TEST_F(FlightStateMachineTest, HandleHoverMission)
{
    // Simulate a mission that transitions to Hover
    drone_sdk::CurrentMission hoverMission = drone_sdk::CurrentMission::HOVER; // Correct use of enum
    fsm.handleNewMission(hoverMission);     // Transition from Airborne to Hover

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::HOVER);
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::HOVER);
}

TEST_F(FlightStateMachineTest, HandleTaskCompleteMission)
{
    // Simulate a task complete mission (could transition to Return Home or similar)
    drone_sdk::CurrentMission taskCompleteMission = drone_sdk::CurrentMission::HOME; // Correct enum for task completion
    fsm.handleNewMission(taskCompleteMission);

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::AIRBORNE);
}

TEST_F(FlightStateMachineTest, HandleSafetyViolationMission)
{
    // Simulate a mission that triggers a safety violation
    drone_sdk::CurrentMission safetyViolationMission = drone_sdk::CurrentMission::EMERGENCY; // Correct enum for emergency
    fsm.handleNewMission(safetyViolationMission);

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::EMERGENCY_LAND);
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::EMERGENCY_LAND);
}

// Test for handling invalid missions or transitions (for example, trying to go airborne from LAND)
TEST_F(FlightStateMachineTest, GotoMissionTransition)
{
    // Simulate an invalid mission (attempt to transition from LANDED to AIRBORNE directly)
    drone_sdk::CurrentMission goToMission = drone_sdk::CurrentMission::GOTO; // This could be invalid, depending on your state machine logic
    fsm.handleNewMission(goToMission);

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::AIRBORNE); // Should not transition
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::AIRBORNE);
}

