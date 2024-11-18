#include "state_machine_manager.hpp"
#include "icd.hpp"

#include <gtest/gtest.h>
#include <queue>
#include <optional>

using namespace drone_sdk;

class MockObserver
{
public:
    // Safety state updates
    void onGpsStateUpdate(safetyState state)
    {
        m_lastGpsState = state;
    }

    void onLinkStateUpdate(safetyState state)
    {
        m_lastLinkState = state;
    }

    void onFlightStateUpdate(FlightState state)
    {
        if (m_times_updated > 0)
        {
            m_prevFlightState = m_lastFlightState;
        }
        m_lastFlightState = state;
        ++m_times_updated;
    }

    // Command state updates
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

    // Getters for Safety State updates
    safetyState getLastGpsState() const { return m_lastGpsState; }
    safetyState getLastLinkState() const { return m_lastLinkState; }

    FlightState getLastFlightState() const { return m_lastFlightState; }
    FlightState getPrevFlightState() const { return m_prevFlightState; }
    size_t getFlightUpdates() const { return m_times_updated; }

    // Getters for Command updates
    CommandStatus getLastState() const { return lastState; }
    Location getLastWaypoint() const { return lastWaypoint; }
    Location getLastDestination() const { return lastDestination; }
    bool isLandingTriggered() const { return landingTriggered; }
    bool isTakingOffTriggered() const { return takingOffTriggered; }

private:
    safetyState m_lastGpsState = safetyState::GPS_HEALTH;
    safetyState m_lastLinkState = safetyState::CONNECTED;

    FlightState m_lastFlightState = FlightState::LANDED;
    FlightState m_prevFlightState = FlightState::LANDED;
    size_t m_times_updated = 0;

    CommandStatus lastState = CommandStatus::IDLE;
    Location lastWaypoint;
    Location lastDestination;
    bool landingTriggered = false;
    bool takingOffTriggered = false;
};

class StateMachineManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        m_stateMachineManager.start();
        // Attach observers for various states
        m_stateMachineManager.subscribeToGpsSignalState(
            [this](safetyState state)
            { m_observer.onGpsStateUpdate(state); });

        m_stateMachineManager.subscribeToLinkSignalState(
            [this](safetyState state)
            { m_observer.onLinkStateUpdate(state); });

        m_stateMachineManager.subscribeToFlightState(
            [this](FlightState state)
            { m_observer.onFlightStateUpdate(state); });

        // Attach observer for CommandMachine states
        m_stateMachineManager.subscribeToCommandState(
            [this](CommandStatus state)
            { m_observer.onStateChanged(state); });

        m_stateMachineManager.subscribeToWaypoint(
            [this](Location waypoint)
            { m_observer.onPathWaypoint(waypoint); });

        m_stateMachineManager.subscribeToCurrentDestination(
            [this](Location destination)
            { m_observer.onCurrentDestination(destination); });

        //        m_stateMachineManager.subscribeToLandingEvent(
        //            [this]()
        //            { m_observer.onLanding(); });

        //        m_stateMachineManager.subscribeToTakingOffEvent(
        //            [this]()
        //            { m_observer.onTakingOff(); });
    }

    StateMachineManager m_stateMachineManager;
    MockObserver m_observer;
};

TEST_F(StateMachineManagerTest, HandleGpsSignalNotHealthy)
{
    // Simulate GPS signal update with NO_SIGNAL
    Location mockLocation{37.7749, 122.4194, 30.0}; // Example location
    SignalQuality mockSignalQuality = SignalQuality::NO_SIGNAL;

    m_stateMachineManager.handleGpsUpdate(mockLocation, mockSignalQuality);

    // Validate the SafetyStateMachine's GPS state and the observer's last received state
    EXPECT_EQ(m_observer.getLastGpsState(), safetyState::GPS_NOT_HEALTHY);
}

// TEST_F(StateMachineManagerTest, HandleLinkSignalNotHealthy)
//{
//     // Simulate Link signal update with NO_SIGNAL
//     SignalQuality mockSignalQuality = SignalQuality::NO_SIGNAL;
//
//     m_stateMachineManager.handleLinkUpdate(mockSignalQuality);
//
//     // Validate the SafetyStateMachine's Link state and the observer's last received state
//     EXPECT_EQ(m_observer.getLastLinkState(), safetyState::NOT_CONNECTED);
// }

// TEST_F(StateMachineManagerTest, HandleWaypointUpdate)
//{
//     // Simulate a waypoint update
//     Location mockWaypoint{37.7749, 122.4194, 30.0}; // Example waypoint
//     m_stateMachineManager.handleWaypointUpdate(mockWaypoint);
//
//     // Validate that the observer's last received waypoint matches the expected one
//     EXPECT_EQ(m_observer.getLastWaypoint(), mockWaypoint);
// }
//
// TEST_F(StateMachineManagerTest, HandleDestinationUpdate)
//{
//     // Simulate a destination update
//     Location mockDestination{37.7750, 122.4195, 30.0}; // Example destination
//     m_stateMachineManager.handleDestinationUpdate(mockDestination);
//
//     // Validate that the observer's last received destination matches the expected one
//     EXPECT_EQ(m_observer.getLastDestination(), mockDestination);
// }
//
// TEST_F(StateMachineManagerTest, HandleLandingTrigger)
//{
//     // Simulate a landing event
//     m_stateMachineManager.handleLandingEvent();
//
//     // Validate that the observer's landing trigger was called
//     EXPECT_TRUE(m_observer.isLandingTriggered());
// }

// TEST_F(StateMachineManagerTest, HandleTakingOffTrigger)
//{
//     // Simulate a taking off event
//     m_stateMachineManager.handleTakingOffEvent();
//
//     // Validate that the observer's taking off trigger was called
//     EXPECT_TRUE(m_observer.isTakingOffTriggered());
// }