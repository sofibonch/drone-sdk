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

//---- goto tests----
// Test GOTO mission assignment
TEST_F(CommandStateMachineTest, AssignGotoMission)
{
    Location destination{10.0, 20.0, 100.0};

    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), destination); // destination is not upfated on goto, only in PATH!!!!
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);
}

// Test GOTO mission assignment completion no interrupt
TEST_F(CommandStateMachineTest, ImmidiateGotoMission)
{
    Location destination{10.0, 20.0, 100.0};

    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), destination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    mockGps.setLocation(10.0, 20.0, 100.0);
    stateMachine.handleGpsLocationUpdate(mockGps.getLocation());

    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
    EXPECT_NE(observer.getLastDestination(), destination);
}

TEST_F(CommandStateMachineTest, LongWayGoToMission)
{
    Location destination{10.0, 20.0, 100.0}; // latitude, longitude, altitude

    // Assign a GOTO mission to the state machine.
    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    // Check if the state machine is in the expected initial state.
    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), destination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    // Simulate multiple GPS updates until the destination is reached.
    for (int i = 0; i < 1001; ++i)
    {
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,   // Latitude moves towards 10.0
            20.0 * progress,   // Longitude moves towards 20.0
            100.0 * progress); // Altitude moves towards 100.0
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());

        if (observer.getLastState() == CommandStatus::IDLE)
        {
            break; // Stop testing as the mission is marked complete.
        }
    }

    // Final assertions: the state should be IDLE, and the destination should match.
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
    EXPECT_NE(observer.getLastDestination(), destination);
}

TEST_F(CommandStateMachineTest, LongWayGoToMissionWithEmergency)
{
    Location destination{10.0, 20.0, 100.0}; // latitude, longitude, altitude

    // Assign a GOTO mission to the state machine.
    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    // Check if the state machine is in the expected initial state.
    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), destination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    // Simulate GPS updates, progressing toward the destination.
    for (int i = 0; i < 500; ++i)
    { // Simulate halfway progress
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,   // Latitude moves towards 10.0
            20.0 * progress,   // Longitude moves towards 20.0
            100.0 * progress); // Altitude moves towards 100.0
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    }

    // Simulate GPS safety state change to GPS_NOT_HEALTHY.
    stateMachine.handleGpsStateChange(drone_sdk::safetyState::GPS_NOT_HEALTHY);

    // Verify that the mission is aborted and the state is updated.
    EXPECT_EQ(observer.getLastState(), CommandStatus::MISSION_ABORT);
    EXPECT_NE(observer.getLastDestination(), destination); // Mission should no longer target the original destination.

    // Ensure further updates are ignored and state remains in MissionAbort.
    for (int i = 501; i < 1000; ++i)
    {
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,
            20.0 * progress,
            100.0 * progress);
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    }

    EXPECT_EQ(observer.getLastState(), CommandStatus::MISSION_ABORT);
}

TEST_F(CommandStateMachineTest, LongWayGoToMissionWithLinkEmergency)
{
    Location destination{10.0, 20.0, 100.0}; // latitude, longitude, altitude

    // Assign a GOTO mission to the state machine.
    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    // Check if the state machine is in the expected initial state.
    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), destination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    // Simulate GPS updates, progressing toward the destination.
    for (int i = 0; i < 500; ++i)
    { // Simulate halfway progress
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,   // Latitude moves towards 10.0
            20.0 * progress,   // Longitude moves towards 20.0
            100.0 * progress); // Altitude moves towards 100.0
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    }

    // Simulate link safety state change to LINK_NOT_HEALTHY.
    stateMachine.handleLinkStateChange(drone_sdk::safetyState::NOT_CONNECTED);

    // Verify that the mission is aborted and the state is updated.
    EXPECT_EQ(observer.getLastState(), CommandStatus::MISSION_ABORT);
    EXPECT_NE(observer.getLastDestination(), destination); // Mission should no longer target the original destination.

    // Ensure further updates are ignored and state remains in MissionAbort.
    for (int i = 501; i < 1000; ++i)
    {
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,
            20.0 * progress,
            100.0 * progress);
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    }

    EXPECT_EQ(observer.getLastState(), CommandStatus::MISSION_ABORT);
}

TEST_F(CommandStateMachineTest, LongWayGoToMissionWithInterruptAndReassignment)
{
    Location initialDestination{10.0, 20.0, 100.0}; // Initial destination
    Location newDestination{30.0, 40.0, 200.0};     // New destination after interruption

    // Assign an initial GOTO mission to the state machine.
    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, initialDestination);

    // Check if the state machine is in the expected initial state.
    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), initialDestination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    // Simulate GPS updates, progressing toward the initial destination.
    for (int i = 0; i < 500; ++i)
    { // Simulate halfway progress
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,   // Latitude moves towards 10.0
            20.0 * progress,   // Longitude moves towards 20.0
            100.0 * progress); // Altitude moves towards 100.0
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    }

    // Simulate an interrupt: Assign a new GOTO mission.
    status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, newDestination);

    // Check if the state machine switched to the new task and remains busy.
    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), newDestination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    mockGps.setLocation(
        0.0,  // Latitude moves from 10.0 to 30.0
        0.0,  // Longitude moves from 20.0 to 40.0
        0.0); // Altitude moves from 100.0 to 200.0
    stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    // Simulate GPS updates, progressing toward the initial destination.
    for (int i = 0; i < 999; ++i)
    { // Simulate halfway progress
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            30.0 * progress,   // Latitude moves towards 10.0
            40.0 * progress,   // Longitude moves towards 20.0
            200.0 * progress); // Altitude moves towards 100.0
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
        // Verify that the state machine remains busy during the mission.
        EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    }
    double progress = static_cast<double>(999) / 999.0;
    mockGps.setLocation(
        30.0 * progress,   // Latitude moves towards 10.0
        40.0 * progress,   // Longitude moves towards 20.0
        200.0 * progress); // Altitude moves towards 100.0
    stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
}

TEST_F(CommandStateMachineTest, LongWayGoToMissionWithHoverInterrupt)
{
    Location destination{10.0, 20.0, 100.0}; // Initial destination

    // Assign a GOTO mission to the state machine.
    FlightControllerStatus status = stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);

    // Check if the state machine is in the expected initial state.
    EXPECT_EQ(observer.getLastState(), CommandStatus::BUSY);
    EXPECT_NE(observer.getLastDestination(), destination);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);

    // Simulate GPS updates, progressing toward the destination.
    for (int i = 0; i < 500; ++i)
    { // Simulate halfway progress
        double progress = static_cast<double>(i) / 999.0;
        mockGps.setLocation(
            10.0 * progress,   // Latitude moves towards 10.0
            20.0 * progress,   // Longitude moves towards 20.0
            100.0 * progress); // Altitude moves towards 100.0
        stateMachine.handleGpsLocationUpdate(mockGps.getLocation());
    }

    // Simulate an interrupt: Assign a HOVER task.
    status = stateMachine.handleTaskAssigned(CurrentMission::HOVER, {});

    // Verify that the state machine immediately transitions to IDLE.
    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
    EXPECT_EQ(status, FlightControllerStatus::SUCCESS);
}

// Test PATH mission waypoint updates with mock GPS (direct update, no callback)
// TEST_F(CommandStateMachineTest, HandlePathWaypointUpdate)
//{
//    std::queue<Location> path;
//    path.push({10.0, 20.0, 100.0});
//    path.push({15.0, 25.0, 150.0});
//
//    stateMachine.handleTaskAssigned(CurrentMission::PATH, std::nullopt, path);
//
//    // Directly update state machine with mock GPS location
//    mockGps.setLocation(10.0, 20.0, 100.0);
//    stateMachine.handleGpsLocationUpdate(mockGps.getLocation()); // Direct update, no callback
//
//    EXPECT_EQ(observer.getLastWaypoint(), path.front());
//    EXPECT_EQ(observer.getLastDestination(), path.back());
//}
//
//// Test emergency landing scenario
// TEST_F(CommandStateMachineTest, HandleEmergencyLanding)
//{
//     Location destination{10.0, 20.0, 100.0};
//     stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);
//
//     // Simulate a GPS signal failure
//     mockGps.setSignalQuality(SignalQuality::NO_SIGNAL);
//     stateMachine.handleLinkStateChange(safetyState::NOT_CONNECTED);
//     stateMachine.handleGpsLocationUpdate(mockGps.getLocation()); // Direct update, no callback
//
//     EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
// }
//
//// Test GPS location updates when receiving signal from mock GPS (direct update, no callback)
// TEST_F(CommandStateMachineTest, HandleGpsLocationUpdate)
//{
//     Location destination{10.0, 20.0, 100.0};
//     stateMachine.handleTaskAssigned(CurrentMission::GOTO, destination);
//
//     // Update GPS location directly in state machine
//     mockGps.setLocation(10.0, 20.0, 100.0);
//     stateMachine.handleGpsLocationUpdate(mockGps.getLocation()); // Direct update, no callback
//
//     EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE); // Should revert to idle state
// }
//
//// Additional tests based on scenarios
//
// TEST_F(CommandStateMachineTest, HandleGotoWithoutDestination)
//{
//    // GOTO with no destination should remain in IDLE state
//    EXPECT_NO_THROW(stateMachine.handleTaskAssigned(CurrentMission::GOTO, std::nullopt));
//    EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
//}
//
//// Handle hover command, should remain idle
// TEST_F(CommandStateMachineTest, HandleHover)
//{
//     stateMachine.handleTaskAssigned(CurrentMission::HOVER, std::nullopt);
//     EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE); // Hover should keep us in IDLE state
// }
//
//// Handle path with an empty list, no destination
// TEST_F(CommandStateMachineTest, HandleEmptyPath)
//{
//     std::queue<Location> emptyPath;
//     stateMachine.handleTaskAssigned(CurrentMission::PATH, std::nullopt, emptyPath);
//     EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE); // No destination should keep it idle
// }
//
//// Test transition to aborted state on safety
// TEST_F(CommandStateMachineTest, HandleAbortedOnSafety)
//{
//     EXPECT_EQ(observer.getLastState(), CommandStatus::IDLE);
// }
//
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