#include "state_machines/command_state_machine.hpp"
#include "../mocks/mock_gps.hpp" // Include the mock GPS
#include "icd.hpp"

#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>
#include <queue>
#include <optional>

// Use the mock GPS
using namespace hw_sdk_mock;
using namespace boost::placeholders;
using namespace commandstatemachine;
using namespace drone_sdk;

// Helper class to observe state changes
class CommandStateChangeObserver
{
public:
    void onStateChanged(CommandStatus newState)
    {
        lastState = newState;
    }

    CommandStatus getLastState() const { return lastState; }

private:
    CommandStatus lastState = CommandStatus::IDLE;
};

// Helper class to observe waypoint updates
class CommandWaypointUpdateObserver
{
public:
    void onWaypointUpdated(const Location &waypoint)
    {
        lastWaypoint = waypoint;
    }

    Location getLastWaypoint() const { return lastWaypoint; }

private:
    Location lastWaypoint;
};

// Test fixture
class CommandStateMachineTest : public ::testing::Test
{
protected:
    CommandStateMachine csm;
    CommandStateChangeObserver stateObserver;
    CommandWaypointUpdateObserver waypointObserver;
    MockGps mockGps;

    void SetUp() override
    {
        // Subscribe the state observer to state change signals
        csm.subscribeToState(
            boost::bind(&CommandStateChangeObserver::onStateChanged, &stateObserver, _1));

        // Subscribe the waypoint observer to waypoint updates
        csm.subscribeToPathWaypoint(
            boost::bind(&CommandWaypointUpdateObserver::onWaypointUpdated, &waypointObserver, _1));
    }
};

// Test: Initial state
TEST_F(CommandStateMachineTest, InitialState)
{
    EXPECT_EQ(csm.getCurrentState(), CommandStatus::IDLE);
}

// Test: Subscription to state changes
TEST_F(CommandStateMachineTest, SubscriptionStateChange)
{
    // Trigger a state change and verify that the state observer gets updated
    csm.handleTaskAssigned(CurrentMission::GOTO, Location{1.0, 2.0, 3.0});
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::BUSY);
}

// Test: Subscription to waypoint updates with an empty path
TEST_F(CommandStateMachineTest, SubscriptionWaypointEmptyPath)
{
    csm.handleTaskAssigned(CurrentMission::PATH, {}, std::queue<Location>());
    // Verify that the waypoint observer is not called when there is no path
    EXPECT_EQ(waypointObserver.getLastWaypoint().latitude, 0.0);
    EXPECT_EQ(waypointObserver.getLastWaypoint().longitude, 0.0);
    EXPECT_EQ(waypointObserver.getLastWaypoint().altitude, 0.0);
    ;
}

// Test: Subscription to waypoint updates with several points
TEST_F(CommandStateMachineTest, SubscriptionWaypointMultiplePoints)
{
    std::queue<Location> path;
    path.push(Location{1.0, 2.0, 3.0});
    path.push(Location{4.0, 5.0, 6.0});

    csm.handleTaskAssigned(CurrentMission::PATH, {}, path);

    // Simulate the drone's progress through the path
    // Update the drone's location to match the first waypoint and check the observer
    csm.handleGpsLocationUpdate(Location{1.0, 2.0, 3.0}); // Simulate reaching the first waypoint
    EXPECT_EQ(waypointObserver.getLastWaypoint().latitude, 1.0);
    EXPECT_EQ(waypointObserver.getLastWaypoint().longitude, 2.0);
    EXPECT_EQ(waypointObserver.getLastWaypoint().altitude, 3.0);

    // Update the drone's location to match the second waypoint and check the observer
    csm.handleGpsLocationUpdate(Location{4.0, 5.0, 6.0}); // Simulate reaching the second waypoint
    
    EXPECT_EQ(waypointObserver.getLastWaypoint().latitude, 4.0);
    EXPECT_EQ(waypointObserver.getLastWaypoint().longitude, 5.0);
    EXPECT_EQ(waypointObserver.getLastWaypoint().altitude, 6.0);
}

// Test: Handle 'GOTO' mission with and without destination
TEST_F(CommandStateMachineTest, HandleGotoMission)
{
    // Test without destination
    EXPECT_THROW(csm.handleTaskAssigned(CurrentMission::GOTO), std::invalid_argument);

    // Test with destination
    csm.handleTaskAssigned(CurrentMission::GOTO, Location{1.0, 2.0, 3.0});
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::BUSY);
}

// Test: Handle 'HOME' mission
TEST_F(CommandStateMachineTest, HandleHomeMission)
{
    csm.setHomebase(Location{0.0, 0.0, 0.0});
    csm.handleTaskAssigned(CurrentMission::HOME);
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::BUSY);
}

// Test: Handle 'HOVER' mission
TEST_F(CommandStateMachineTest, HandleHoverMission)
{
    csm.handleTaskAssigned(CurrentMission::HOVER);
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::BUSY);
}

// Test: Handle 'PATH' mission with empty path
TEST_F(CommandStateMachineTest, HandlePathMissionEmptyPath)
{
    csm.handleTaskAssigned(CurrentMission::PATH, {}, std::queue<Location>());
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::BUSY);
}

// Test: Handle aborted task due to safety
TEST_F(CommandStateMachineTest, HandleAbortedTaskSafety)
{
    csm.handleTaskAssigned(CurrentMission::GOTO, Location{1.0, 2.0, 3.0});
    csm.handleTaskAbortedSafety();
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::MISSION_ABORT);
}

// Test: GPS location update with mock GPS
TEST_F(CommandStateMachineTest, HandleGpsLocationUpdate)
{
    csm.handleTaskAssigned(CurrentMission::GOTO, Location{1.0, 2.0, 3.0});
    mockGps.setLocation(1.0, 2.0, 3.0);                 // Set the mock GPS location
    mockGps.setSignalQuality(SignalQuality::EXCELLENT); // Set signal quality

    // Simulate GPS location update
    csm.handleGpsLocationUpdate(mockGps.getLocation());
    EXPECT_EQ(stateObserver.getLastState(), CommandStatus::MISSION_COMPLETE);
}

// Test: Set home base location
TEST_F(CommandStateMachineTest, SetHomeBase)
{
    Location home{0.0, 0.0, 0.0};
    csm.setHomebase(home);
    EXPECT_EQ(csm.getHomebase(), home);
}

// Test: Get current state
TEST_F(CommandStateMachineTest, GetCurrentState)
{
    EXPECT_EQ(csm.getCurrentState(), CommandStatus::IDLE);
    csm.handleTaskAssigned(CurrentMission::GOTO, Location{1.0, 2.0, 3.0});
    EXPECT_EQ(csm.getCurrentState(), CommandStatus::BUSY);
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