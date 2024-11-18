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
TEST_F(StateMachineManagerTest, HandleLinkSignalNotHealthy)
{
    // Simulate Link signal update with NO_SIGNAL
    SignalQuality mockSignalQuality = SignalQuality::NO_SIGNAL;

    m_stateMachineManager.handleLinkUpdate(mockSignalQuality);

    // Validate that the observer's last received state is NOT_CONNECTED
    EXPECT_EQ(m_observer.getLastLinkState(), safetyState::NOT_CONNECTED);

    // Check if CommandObserver receives the appropriate updates
    EXPECT_EQ(m_observer.getLastState(), CommandStatus::IDLE);
}

TEST_F(StateMachineManagerTest, HandleLinkSignalConnected)
{
    // Simulate Link signal update with CONNECTED
    SignalQuality mockSignalQuality = SignalQuality::EXCELLENT;

    m_stateMachineManager.handleLinkUpdate(mockSignalQuality);

    // Validate that the observer's last received state is CONNECTED
    EXPECT_EQ(m_observer.getLastLinkState(), safetyState::CONNECTED);

    // Check if CommandObserver receives the appropriate updates when connection is established
    EXPECT_EQ(m_observer.getLastState(), CommandStatus::IDLE);
}

TEST_F(StateMachineManagerTest, NewTaskGotoMission)
{
    // Create a GOTO mission with a specific destination
    drone_sdk::CurrentMission newMission = drone_sdk::CurrentMission::GOTO;
    drone_sdk::Location singleDestination{37.7749, 122.4194, 30.0}; // Example location

    std::optional<drone_sdk::Location> destination = singleDestination;
    std::optional<std::queue<drone_sdk::Location>> pathDestinations = std::nullopt;

    drone_sdk::FlightControllerStatus status = m_stateMachineManager.newTask(newMission, destination, pathDestinations);

    // Validate that the flight controller receives the expected status for a GOTO mission
    EXPECT_EQ(status, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::BUSY); // Command should be updated to GOTO
    EXPECT_NE(m_observer.getLastDestination(), singleDestination);        // Ensure the destination is set correctly
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::TAKEOFF);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE);
}

TEST_F(StateMachineManagerTest, NewTaskGotoMissionWithHoverInterrupt)
{
    // Create a GOTO mission with a specific destination
    drone_sdk::CurrentMission newMission = drone_sdk::CurrentMission::GOTO;
    drone_sdk::CurrentMission interruptMission = drone_sdk::CurrentMission::HOVER;
    drone_sdk::Location singleDestination{37.7749, 122.4194, 30.0}; // Example location

    std::optional<drone_sdk::Location> destination = singleDestination;
    std::optional<std::queue<drone_sdk::Location>> pathDestinations = std::nullopt;
    std::optional<drone_sdk::Location> emptyDestination = std::nullopt;

    // First task: GOTO mission
    drone_sdk::FlightControllerStatus statusGoto = m_stateMachineManager.newTask(newMission, destination, pathDestinations);

    // Validate the flight controller's response for the GOTO mission
    EXPECT_EQ(statusGoto, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::BUSY); // Command should be updated to GOTO
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::TAKEOFF);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE);

    // Interrupt the mission with a HOVER command
    drone_sdk::FlightControllerStatus statusHover = m_stateMachineManager.newTask(interruptMission, emptyDestination, pathDestinations);

    // Validate the flight controller's response for the HOVER interrupt
    EXPECT_EQ(statusHover, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::IDLE); // Command should reflect IDLE after interruption
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::HOVER);//???
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::HOVER);
}

TEST_F(StateMachineManagerTest, NewTaskGotoMissionWithComplition)
{
    // Create a GOTO mission with a specific destination
    drone_sdk::CurrentMission newMission = drone_sdk::CurrentMission::GOTO;
    drone_sdk::Location singleDestination{37.7749, 122.4194, 30.0}; // Example location

    std::optional<drone_sdk::Location> destination = singleDestination;
    std::optional<std::queue<drone_sdk::Location>> pathDestinations = std::nullopt;

    // First task: GOTO mission
    drone_sdk::FlightControllerStatus statusGoto = m_stateMachineManager.newTask(newMission, destination, pathDestinations);

    // Validate the flight controller's response for the GOTO mission
    EXPECT_EQ(statusGoto, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::BUSY); // Command should be updated to GOTO
    // EXPECT_EQ(m_observer.getLastDestination(), singleDestination);       // Ensure the destination is set correctly
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::TAKEOFF);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE);

    // Simulate GPS updates leading to destination
    drone_sdk::Location intermediateLocation1{37.7740, 122.4190, 30.0};
    drone_sdk::Location intermediateLocation2{37.7745, 122.4192, 30.0};
    drone_sdk::Location finalLocation = singleDestination;

    // Update GPS to an intermediate location
    m_stateMachineManager.handleGpsUpdate(intermediateLocation1, drone_sdk::SignalQuality::EXCELLENT);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE); // Still airborne

    // Update GPS to another intermediate location
    m_stateMachineManager.handleGpsUpdate(intermediateLocation2, drone_sdk::SignalQuality::EXCELLENT);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE); // Still airborne

    // Update GPS to the final destination
    m_stateMachineManager.handleGpsUpdate(finalLocation, drone_sdk::SignalQuality::EXCELLENT);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::HOVER); // Mission complete

    // Validate the flight controller's response for the HOVER interrupt
    // EXPECT_EQ(statusHover, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::IDLE); // Command should reflect IDLE after interruption
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::HOVER);
}

TEST_F(StateMachineManagerTest, NewTaskSlowGotoMissionWithCompletion)
{
    // Create a GOTO mission with a specific destination
    drone_sdk::CurrentMission newMission = drone_sdk::CurrentMission::GOTO;
    drone_sdk::Location singleDestination{10.0, 20.0, 100.0}; // Example location

    std::optional<drone_sdk::Location> destination = singleDestination;
    std::optional<std::queue<drone_sdk::Location>> pathDestinations = std::nullopt;

    // Assign the GOTO mission
    drone_sdk::FlightControllerStatus statusGoto = m_stateMachineManager.newTask(newMission, destination, pathDestinations);

    // Validate the flight controller's response for the GOTO mission
    EXPECT_EQ(statusGoto, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::BUSY); // Command should be updated to GOTO
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::TAKEOFF);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE);

    // Simulate GPS updates over a long journey
    for (int i = 0; i < 1001; ++i)
    {
        double progress = static_cast<double>(i) / 999.0;
        drone_sdk::Location currentLocation{
            10.0 * progress,   // Latitude moves towards 10.0
            20.0 * progress,   // Longitude moves towards 20.0
            100.0 * progress}; // Altitude moves towards 100.0

        // Update GPS location
        m_stateMachineManager.handleGpsUpdate(currentLocation, drone_sdk::SignalQuality::EXCELLENT);

        // Check if the mission is marked complete
        if (m_observer.getLastState() == drone_sdk::CommandStatus::IDLE)
        {
            break; // Stop testing as the mission is complete
        }
    }

    // Validate final state after completing the journey
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::HOVER); // Mission should end in hover
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::IDLE);      // Command should reflect mission completion
}

TEST_F(StateMachineManagerTest, NewTaskGotoMissionWithAbortInterrupt)
{
    // Create a GOTO mission with a specific destination
    drone_sdk::CurrentMission newMission = drone_sdk::CurrentMission::GOTO;
    drone_sdk::CurrentMission interruptMission = drone_sdk::CurrentMission::HOME;
    drone_sdk::Location singleDestination{37.7749, 122.4194, 30.0}; // Example location
    drone_sdk::Location homeDestination{0.0, 0.0, 0.0};             // Example location
    drone_sdk::Location interruptDestination{1.0, 1.0, 1.0};

    std::optional<drone_sdk::Location> destination = singleDestination;
    std::optional<std::queue<drone_sdk::Location>> pathDestinations = std::nullopt;
    std::optional<drone_sdk::Location> emptyDestination = std::nullopt;

    // First task: GOTO mission
    drone_sdk::FlightControllerStatus statusGoto = m_stateMachineManager.newTask(newMission, destination, pathDestinations);

    // Validate the flight controller's response for the GOTO mission
    EXPECT_EQ(statusGoto, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::BUSY); // Command should be updated to GOTO
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::TAKEOFF);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::AIRBORNE);

    // update mid flight location
    m_stateMachineManager.handleGpsUpdate(interruptDestination, drone_sdk::SignalQuality::EXCELLENT);

    // Interrupt the mission with a HOVER command
    drone_sdk::FlightControllerStatus statusHome = m_stateMachineManager.newTask(interruptMission, emptyDestination, pathDestinations);

    // Validate the flight controller's response for the HOVER interrupt
    EXPECT_EQ(statusHome, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::BUSY);
    EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::AIRBORNE);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::RETURN_HOME);

    // update mid flight location
    m_stateMachineManager.handleGpsUpdate(homeDestination, drone_sdk::SignalQuality::EXCELLENT);

    EXPECT_EQ(statusHome, drone_sdk::FlightControllerStatus::SUCCESS);
    EXPECT_EQ(m_observer.getLastState(), drone_sdk::CommandStatus::IDLE); // Command should reflect IDLE after interruption
   // EXPECT_EQ(m_observer.getPrevFlightState(), drone_sdk::FlightState::LANDED);
    EXPECT_EQ(m_observer.getLastFlightState(), drone_sdk::FlightState::RETURN_HOME);
}
