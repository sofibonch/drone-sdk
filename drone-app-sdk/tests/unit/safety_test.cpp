#include <gtest/gtest.h>
#include <memory>
#include "state_machines/safety_state_machine.hpp"

class SafetyStateMachineTest : public ::testing::Test {
protected:
    safetystatemachine::SafetyStateMachine m_safetyStateMachine;

    // Captured states for GPS and Link
    drone_sdk::safetyState m_gpsState;
    drone_sdk::safetyState m_linkState;

    SafetyStateMachineTest()
        : m_gpsState(drone_sdk::safetyState::GPS_HEALTH),
          m_linkState(drone_sdk::safetyState::CONNECTED) {
        // Subscribe to GPS state changes
        m_safetyStateMachine.subscribeToGpsState(
            [this](drone_sdk::safetyState state) { m_gpsState = state; });

        // Subscribe to Link state changes
        m_safetyStateMachine.subscribeToLinkState(
            [this](drone_sdk::safetyState state) { m_linkState = state; });
    }
};

// Test GPS signal transitions
TEST_F(SafetyStateMachineTest, GpsSignalTransitions) {
    safetystatemachine::GpsSignal gpsSignal;
   
    // Start with a healthy GPS signal
    gpsSignal.quality = drone_sdk::SignalQuality::GOOD;
    m_safetyStateMachine.handleGpsSignal(gpsSignal);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_HEALTH);

    // Transition to GPS not healthy
    gpsSignal.quality = drone_sdk::SignalQuality::NO_SIGNAL;
    m_safetyStateMachine.handleGpsSignal(gpsSignal);
    m_gpsState=m_safetyStateMachine.getCurrentGpsState();
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_NOT_HEALTHY);

    // Transition back to GPS healthy
    gpsSignal.quality = drone_sdk::SignalQuality::EXCELLENT;
    m_safetyStateMachine.handleGpsSignal(gpsSignal);
    m_gpsState=m_safetyStateMachine.getCurrentGpsState();
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_HEALTH);
}

// Test Link signal transitions
TEST_F(SafetyStateMachineTest, LinkSignalTransitions) {
    safetystatemachine::LinkSignal linkSignal;
    // Start with a connected link signal
    linkSignal.quality = drone_sdk::SignalQuality::GOOD;
    m_safetyStateMachine.handleLinkSignal(linkSignal);
    m_linkState=m_safetyStateMachine.getCurrentLinkState();
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::CONNECTED);

    // Transition to link disconnected
    linkSignal.quality = drone_sdk::SignalQuality::NO_SIGNAL;
    m_safetyStateMachine.handleLinkSignal(linkSignal);
    m_linkState=m_safetyStateMachine.getCurrentLinkState();
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::NOT_CONNECTED);

    // Even if the signal improves, the link remains disconnected
    linkSignal.quality = drone_sdk::SignalQuality::FAIR;
    m_safetyStateMachine.handleLinkSignal(linkSignal);
    m_linkState=m_safetyStateMachine.getCurrentLinkState();
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::CONNECTED);
}
