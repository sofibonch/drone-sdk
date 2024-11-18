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
        // there is an option of updating from landed to hover in achain with takeoff
        if (times_updated > 0)
        {
            prevState = lastState;
        }
        lastState = newState;
        ++times_updated;
    }

    drone_sdk::FlightState getLastState() const { return lastState; }
    drone_sdk::FlightState getPrevState() const { return prevState; }

    size_t getTransitionCount() { return times_updated; }

private:
    drone_sdk::FlightState prevState = drone_sdk::FlightState::LANDED;
    drone_sdk::FlightState lastState = drone_sdk::FlightState::LANDED;
    size_t times_updated = 0;
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

//-- tests for handeling landed mission --
TEST_F(FlightStateMachineTest, HandleTaskLandedToGoto)
{
    // Simulate a task complete mission (could transition to Return Home or similar)
    drone_sdk::CurrentMission taskCompleteMission = drone_sdk::CurrentMission::GOTO; // Correct enum for task completion
    fsm.handleNewMission(taskCompleteMission);

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(observer.getPrevState(), drone_sdk::FlightState::TAKEOFF);
    EXPECT_TRUE(observer.getTransitionCount()== 2);
}

//-- tests for handeling TakeOff mission --
//-- tests for handeling AirBorn mission --
//-- tests for handeling Hover mission --
//-- tests for handeling emergancy mission --
//-- tests for handeling home mission --

TEST_F(FlightStateMachineTest, HandleAirborneMission)
{
    // Simulate a mission that transitions to Airborne
    drone_sdk::CurrentMission airborneMission = drone_sdk::CurrentMission::GOTO; // Correct use of enum
    fsm.handleNewMission(airborneMission);                                       // Transition from Takeoff to Airborne

    EXPECT_EQ(fsm.getCurrentState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(observer.getLastState(), drone_sdk::FlightState::AIRBORNE);
}

TEST_F(FlightStateMachineTest, HandleHoverMission)
{
    // Simulate a mission that transitions to Hover
    drone_sdk::CurrentMission hoverMission = drone_sdk::CurrentMission::HOVER; // Correct use of enum
    fsm.handleNewMission(hoverMission);                                        // Transition from Airborne to Hover

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
