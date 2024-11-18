#include "state_machines/safety_state_machine.hpp"
#include "icd.hpp"
#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;
using namespace safetystatemachine;
using namespace drone_sdk;

// Helper class to capture state changes
class SafetyStateChangeObserver
{
public:
    void onGpsStateChanged(safetyState newState)
    {
        m_lastGpsState = newState;
    }

    void onLinkStateChanged(safetyState newState)
    {
        m_lastLinkState = newState;
    }

    safetyState getLastGpsState() const { return m_lastGpsState; }
    safetyState getLastLinkState() const { return m_lastLinkState; }

private:
    safetyState m_lastGpsState = safetyState::GPS_HEALTH;
    safetyState m_lastLinkState = safetyState::CONNECTED;
};

class SafetyStateMachineTest : public ::testing::Test
{
protected:
    SafetyStateMachine sm;
    SafetyStateChangeObserver observer;

    void SetUp() override
    {
        // Subscribe the observer to the state machine
        sm.subscribeToGpsState(
            boost::bind(&SafetyStateChangeObserver::onGpsStateChanged, &observer, _1));
        sm.subscribeToLinkState(
            boost::bind(&SafetyStateChangeObserver::onLinkStateChanged, &observer, _1));
    }
};

// Test: Initial State
TEST_F(SafetyStateMachineTest, InitialState)
{
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);
}

// Section: Tests for handleGpsSignal()
TEST_F(SafetyStateMachineTest, HandleGpsSignalNotHealthy)
{
    SignalQuality gpsSignal = SignalQuality::NO_SIGNAL;
    sm.handleGpsSignal(gpsSignal); // GPS should become not healthy

    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_NOT_HEALTHY);
}

TEST_F(SafetyStateMachineTest, HandleGpsSignalHealthy)
{
    SignalQuality gpsSignal{SignalQuality::EXCELLENT};
    sm.handleGpsSignal(gpsSignal); // GPS should be healthy

    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_HEALTH);
}

// Section: Tests for handleLinkSignal()
TEST_F(SafetyStateMachineTest, HandleLinkSignalDisconnected)
{
    SignalQuality linkSignal{SignalQuality::NO_SIGNAL};
    sm.handleLinkSignal(linkSignal); // Link should become disconnected

    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);
    EXPECT_EQ(observer.getLastLinkState(), safetyState::NOT_CONNECTED);
}

TEST_F(SafetyStateMachineTest, HandleLinkSignalConnected)
{
    SignalQuality linkSignal{SignalQuality::EXCELLENT};
    sm.handleLinkSignal(linkSignal); // Link should be connected

    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);
    EXPECT_EQ(observer.getLastLinkState(), safetyState::CONNECTED);
}

// Section: Tests for transitions from GpsNotHealthy to GpsHealthy
TEST_F(SafetyStateMachineTest, GpsStateRestored)
{
    SignalQuality gpsSignalLost{SignalQuality::NO_SIGNAL};
    sm.handleGpsSignal(gpsSignalLost); // Lost signal -> GPS_NOT_HEALTHY
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);

    SignalQuality gpsSignalRestored{SignalQuality::EXCELLENT};
    sm.handleGpsSignal(gpsSignalRestored); // Ensure no restoration
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_NOT_HEALTHY);
}

// Section: Tests for transitions from ConnectionDisconnected to ConnectionConnected
TEST_F(SafetyStateMachineTest, LinkStateRestored)
{
    SignalQuality linkSignalLost{SignalQuality::NO_SIGNAL};
    sm.handleLinkSignal(linkSignalLost); // Lost connection -> NOT_CONNECTED
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);

    SignalQuality linkSignalRestored{SignalQuality::EXCELLENT};
    sm.handleLinkSignal(linkSignalRestored); // Ensure no restoration
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);
    EXPECT_EQ(observer.getLastLinkState(), safetyState::NOT_CONNECTED);
}

TEST_F(SafetyStateMachineTest, GpsSignalPoorSignal)
{
    SignalQuality gpsSignalPoor{SignalQuality::POOR};
    sm.handleGpsSignal(gpsSignalPoor); // GPS should remain GPS_HEALTH
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_HEALTH);
}

// Section: Combination state tests
TEST_F(SafetyStateMachineTest, CombinedGpsAndLinkSignals_NoRestoration) {
    // Initially both are healthy
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);

    // GPS lost, link weak
    sm.handleGpsSignal(SignalQuality::NO_SIGNAL);
    sm.handleLinkSignal(SignalQuality::POOR);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);

    // Both lost
    sm.handleLinkSignal(SignalQuality::NO_SIGNAL);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);

    // Attempt to restore signals
    sm.handleGpsSignal(SignalQuality::EXCELLENT);
    sm.handleLinkSignal(SignalQuality::EXCELLENT);

    // Verify that states do not restore
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);  // GPS state remains degraded
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);  // Link state remains degraded
}

// Section: GPS Signal Quality Tests
TEST_F(SafetyStateMachineTest, HandleAllGpsSignalQualities)
{
    // Test POOR
    sm.handleGpsSignal(SignalQuality::POOR);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);

    // Test FAIR
    sm.handleGpsSignal(SignalQuality::FAIR);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);

    // Test GOOD
    sm.handleGpsSignal(SignalQuality::GOOD);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);

    // Test EXCELLENT
    sm.handleGpsSignal(SignalQuality::EXCELLENT);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    // Test NO_SIGNAL
    sm.handleGpsSignal(SignalQuality::NO_SIGNAL);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);
}

// Section: Link Signal Quality Tests
TEST_F(SafetyStateMachineTest, HandleAllLinkSignalQualities)
{
    // Test POOR
    sm.handleLinkSignal(SignalQuality::POOR);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);

    // Test FAIR
    sm.handleLinkSignal(SignalQuality::FAIR);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);

    // Test GOOD
    sm.handleLinkSignal(SignalQuality::GOOD);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);

    // Test EXCELLENT
    sm.handleLinkSignal(SignalQuality::EXCELLENT);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);
    // Test NO_SIGNAL
    sm.handleLinkSignal(SignalQuality::NO_SIGNAL);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);
}

// Section: Repeated Signal Tests
TEST_F(SafetyStateMachineTest, RepeatedGpsSignal)
{
    sm.handleGpsSignal(SignalQuality::EXCELLENT);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);

    sm.handleGpsSignal(SignalQuality::EXCELLENT);
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH); // No change expected
}

TEST_F(SafetyStateMachineTest, RepeatedLinkSignal)
{
    sm.handleLinkSignal(SignalQuality::EXCELLENT);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);

    sm.handleLinkSignal(SignalQuality::EXCELLENT);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED); // No change expected
}
