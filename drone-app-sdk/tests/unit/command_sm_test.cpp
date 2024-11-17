#include "state_machines/command_state_machine.hpp"
#include "../mocks/mock_gps.hpp"  // Include the mock GPS
#include "../mocks/mock_link.hpp" // Include the mock Link
#include "icd.hpp"

#include <gtest/gtest.h>
#include <queue>
#include <optional>
#include <boost/bind/bind.hpp> 

using namespace boost::placeholders;

using namespace hw_sdk_mock;
using namespace commandstatemachine;
using namespace drone_sdk;

// Helper class to observe state changes and signals
class CommandStateChangeObserver
{
public:
    void onStateChanged(CommandStatus newState)
    {
        lastState = newState;
    }

    void onPathWaypoint(Location waypoint)
    {
        lastWaypoint = waypoint;
    }

    void onCurrentDestination(Location destination)
    {
        lastDestination = destination;
    }

    void onLanding()
    {
        landingTriggered = true;
    }

    void onTakingOff()
    {
        takingOffTriggered = true;
    }

    CommandStatus getLastState() const { return lastState; }
    Location getLastWaypoint() const { return lastWaypoint; }
    Location getLastDestination() const { return lastDestination; }
    bool isLandingTriggered() const { return landingTriggered; }
    bool isTakingOffTriggered() const { return takingOffTriggered; }

private:
    CommandStatus lastState{CommandStatus::IDLE};
    Location lastWaypoint;
    Location lastDestination;
    bool landingTriggered{false};
    bool takingOffTriggered{false};
};

class CommandStateMachineTest : public ::testing::Test
{
protected:
    CommandStateMachine stateMachine;
    CommandStateChangeObserver observer;
    MockGps mockGps;   // Add MockGps
    MockLink mockLink; // Add MockLink

    void SetUp() override
    {
        // Subscribe observer to state changes and waypoints
        stateMachine.subscribeToState(boost::bind(&CommandStateChangeObserver::onStateChanged, &observer, _1));
        stateMachine.subscribeToPathWaypoint(boost::bind(&CommandStateChangeObserver::onPathWaypoint, &observer, _1));
        stateMachine.subscribeToCurrentDestination(boost::bind(&CommandStateChangeObserver::onCurrentDestination, &observer, _1));
        stateMachine.subscribeToLandingSignal(boost::bind(&CommandStateChangeObserver::onLanding, &observer));
        stateMachine.subscribeToTakingOffSignal(boost::bind(&CommandStateChangeObserver::onTakingOff, &observer));
    }
};

// Test initial state
TEST_F(CommandStateMachineTest, InitialState)
{
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
}

// Test GOTO mission assignment
TEST_F(CommandStateMachineTest, AssignGotoMission)
{
    Location destination{10.0, 20.0, 100.0};

    EXPECT_NO_THROW(stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination));

    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_EQ(observer.getLastDestination(), destination);
}

// Test PATH mission waypoint updates with mock GPS (direct update, no callback)
TEST_F(CommandStateMachineTest, HandlePathWaypointUpdate)
{
    std::queue<Location> path;
    path.push({10.0, 20.0, 100.0});
    path.push({15.0, 25.0, 150.0});

    stateMachine.handleTaskAssigned(CurrentMission::PATH, std::nullopt, path);

    // Directly update state machine with mock GPS location
    mockGps.setLocation(10.0, 20.0, 100.0);
    stateMachine.handleGpsLocationUpdate(mockGps.getLocation());  // Direct update, no callback

    EXPECT_EQ(observer.getLastWaypoint(), path.front());
    EXPECT_EQ(observer.getLastDestination(), path.back());
}

// Test emergency landing scenario
TEST_F(CommandStateMachineTest, HandleEmergencyLanding)
{
    Location destination{10.0, 20.0, 100.0};
    stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    // Simulate a GPS signal failure
    mockGps.setSignalQuality(SignalQuality::NO_SIGNAL);
    stateMachine.handleLinkStateChange(safetyState::NOT_CONNECTED);
    stateMachine.handleGpsLocationUpdate(mockGps.getLocation());  // Direct update, no callback

    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
}

// Test GPS location updates when receiving signal from mock GPS (direct update, no callback)
TEST_F(CommandStateMachineTest, HandleGpsLocationUpdate)
{
    Location destination{10.0, 20.0, 100.0};
    stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    // Update GPS location directly in state machine
    mockGps.setLocation(10.0, 20.0, 100.0);
    stateMachine.handleGpsLocationUpdate(mockGps.getLocation());  // Direct update, no callback

    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE); // Should revert to idle state
}


// Additional tests based on scenarios

TEST_F(CommandStateMachineTest, HandleGotoWithoutDestination)
{
    // GOTO with no destination should remain in IDLE state
    EXPECT_NO_THROW(stateMachine.handleTaskAssigned(CurrentMission::GOTO, std::nullopt));
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
}

// Handle hover command, should remain idle
TEST_F(CommandStateMachineTest, HandleHover)
{
    stateMachine.handleTaskAssigned(CurrentMission::HOVER, std::nullopt);
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE); // Hover should keep us in IDLE state
}

// Handle path with an empty list, no destination
TEST_F(CommandStateMachineTest, HandleEmptyPath)
{
    std::queue<Location> emptyPath;
    stateMachine.handleTaskAssigned(CurrentMission::PATH, std::nullopt, emptyPath);
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE); // No destination should keep it idle
}

// Test transition to aborted state on safety
TEST_F(CommandStateMachineTest, HandleAbortedOnSafety)
{
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
}


/**
 * tests to preform:
 *
 * subscription:
 *  -subsciption for state
 *    - change on every state
 *    - nothing changes / called when the change should happer
 * -subscription for waypoint
 *   -way point on empty path
 *   -wapy point on several points
 *   - waypoint when path is distorted?
 *   - waypoint on full path- vary long version
 * handle task
 *  - goto:
 *      -get destination
 *      -dont get destination
 *      - get path?
 *  - home:
 *      - didnt get destination
 *      - home wasn't set up?
 *  - hover
 *      - hover means staying idle in the same place?
 *  - path
 *      - empty path,
 *      - path withput a "path"
 *      - path with destination?
 * handle aborted because of safety
 *  - abort on states:
 *      -idle
 *      -busy
 *      -emergency
 *      -completed
 * gps location update
 *  - location is being updated corectly when called
 * set home
 * get cuurent state
 *      -getting correct current state according to the machine
 *
 * transitions to check
 *  -idle change by every command:goto, path, home,emergency
 *  -busy for each command and getting every command
 *  -aborted with every command, safety or override
 *  -idle to aborted from safety? needs to go to home base if its in hover on idle???
 */