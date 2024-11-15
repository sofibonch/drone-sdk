#include <gtest/gtest.h>
#include <memory>
#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"

// Test fixture for SafetyStateMachine
class SafetyStateMachineTest : public ::testing::Test {
protected:
    std::shared_ptr<flightstatemachine::FlightStateMachine> m_flightStateMachine;
    safetystatemachine::SafetyStateMachine m_safetyStateMachine;

    // Signals to capture state changes for GPS and Link
    drone_sdk::safetyState m_gpsState;
    drone_sdk::safetyState m_linkState;

    // Set up the test environment
    SafetyStateMachineTest()
        : m_flightStateMachine(std::make_shared<flightstatemachine::FlightStateMachine>()),
          m_safetyStateMachine(m_flightStateMachine),
          m_gpsState(drone_sdk::safetyState::GPS_HEALTH),
          m_linkState(drone_sdk::safetyState::CONNECTED) {
        // Connect state change signals to capture methods
        m_safetyStateMachine.subscribeToGpsState([this](drone_sdk::safetyState state) {
            m_gpsState = state;
        });
        m_safetyStateMachine.subscribeToLinkState([this](drone_sdk::safetyState state) {
            m_linkState = state;
        });
    }
};

// Test GPS signal transition: Healthy -> Not Healthy -> Healthy
TEST_F(SafetyStateMachineTest, GpsSignalTransitions) {
    // Start with a healthy GPS signal
    safetystatemachine::GpsSignal gpsSignal;
    gpsSignal.quality = drone_sdk::SignalQuality::GOOD;
    m_safetyStateMachine.handleGpsSignal(gpsSignal);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_HEALTH);

    // Simulate loss of GPS signal
    gpsSignal.quality = drone_sdk::SignalQuality::NO_SIGNAL;
    m_safetyStateMachine.handleGpsSignal(gpsSignal);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_NOT_HEALTHY);

    // Restore GPS signal
    gpsSignal.quality = drone_sdk::SignalQuality::EXCELLENT;
    m_safetyStateMachine.handleGpsSignal(gpsSignal);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_HEALTH);
}

// Test Link signal transition: Connected -> Disconnected -> Disconnected : reconnecting isn't an option
TEST_F(SafetyStateMachineTest, LinkSignalTransitions) {
    // Start with a connected link signal
    safetystatemachine::LinkSignal linkSignal;
    linkSignal.quality = drone_sdk::SignalQuality::GOOD;
    m_safetyStateMachine.handleLinkSignal(linkSignal);
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::CONNECTED);

    // Simulate loss of Link signal
    linkSignal.quality = drone_sdk::SignalQuality::NO_SIGNAL;
    m_safetyStateMachine.handleLinkSignal(linkSignal);
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::NOT_CONNECTED);

    // Restore Link signal
    linkSignal.quality = drone_sdk::SignalQuality::FAIR;
    m_safetyStateMachine.handleLinkSignal(linkSignal);
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::NOT_CONNECTED);
}
