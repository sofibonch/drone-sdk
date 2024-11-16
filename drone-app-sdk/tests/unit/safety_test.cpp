#include "state_machines/safety_state_machine.hpp"
#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;
using namespace safetystatemachine;
using namespace drone_sdk;

// Helper class to capture state changes
class SafetyStateChangeObserver {
public:
    void onGpsStateChanged(safetyState newState) {
        lastGpsState = newState;
    }

    void onLinkStateChanged(safetyState newState) {
        lastLinkState = newState;
    }

    safetyState getLastGpsState() const { return lastGpsState; }
    safetyState getLastLinkState() const { return lastLinkState; }



private:
    safetyState lastGpsState = safetyState::GPS_HEALTH;
    safetyState lastLinkState = safetyState::CONNECTED;
};

class SafetyStateMachineTest : public ::testing::Test {
protected:
    SafetyStateMachine sm;
    SafetyStateChangeObserver observer;

    void SetUp() override {
        // Subscribe the observer to the state machine
        sm.subscribeToGpsState(
            boost::bind(&SafetyStateChangeObserver::onGpsStateChanged, &observer, _1));
        sm.subscribeToLinkState(
            boost::bind(&SafetyStateChangeObserver::onLinkStateChanged, &observer, _1));
    }
};

// Test: Initial State
TEST_F(SafetyStateMachineTest, InitialState) {
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);
}

// Section: Tests for handleGpsSignal()
TEST_F(SafetyStateMachineTest, HandleGpsSignalNotHealthy) {
    GpsSignal gpsSignal{SignalQuality::NO_SIGNAL};
    sm.handleGpsSignal(gpsSignal);

    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_NOT_HEALTHY);
    
}

TEST_F(SafetyStateMachineTest, HandleGpsSignalHealthy) {
    GpsSignal gpsSignal{SignalQuality::EXCELLENT};
    sm.handleGpsSignal(gpsSignal);

    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_HEALTH);
    
}

// Section: Tests for handleLinkSignal()
TEST_F(SafetyStateMachineTest, HandleLinkSignalDisconnected) {
    LinkSignal linkSignal{SignalQuality::NO_SIGNAL};
    sm.handleLinkSignal(linkSignal);

    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);
    EXPECT_EQ(observer.getLastLinkState(), safetyState::NOT_CONNECTED);
    
}

TEST_F(SafetyStateMachineTest, HandleLinkSignalConnected) {
    LinkSignal linkSignal{SignalQuality::EXCELLENT};
    sm.handleLinkSignal(linkSignal);

    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);
    EXPECT_EQ(observer.getLastLinkState(), safetyState::CONNECTED);
    
}

// Section: Tests for transitions from GpsNotHealthy to GpsHealthy
TEST_F(SafetyStateMachineTest, GpsStateRestored) {
    GpsSignal gpsSignalLost{SignalQuality::NO_SIGNAL};
    sm.handleGpsSignal(gpsSignalLost);  // Lost signal -> GPS_NOT_HEALTHY
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_NOT_HEALTHY);

    GpsSignal gpsSignalRestored{SignalQuality::EXCELLENT};
    sm.handleGpsSignal(gpsSignalRestored);  // Restored signal -> GPS_HEALTH
    EXPECT_EQ(sm.getCurrentGpsState(), safetyState::GPS_HEALTH);
    EXPECT_EQ(observer.getLastGpsState(), safetyState::GPS_HEALTH);
    
}

// Section: Tests for transitions from ConnectionDisconnected to ConnectionConnected
TEST_F(SafetyStateMachineTest, LinkStateRestored) {
    LinkSignal linkSignalLost{SignalQuality::NO_SIGNAL};
    sm.handleLinkSignal(linkSignalLost);  // Lost connection -> NOT_CONNECTED
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::NOT_CONNECTED);

    LinkSignal linkSignalRestored{SignalQuality::EXCELLENT};
    sm.handleLinkSignal(linkSignalRestored);  // Restored connection -> CONNECTED
    EXPECT_EQ(sm.getCurrentLinkState(), safetyState::CONNECTED);
    EXPECT_EQ(observer.getLastLinkState(), safetyState::CONNECTED);
    
}

