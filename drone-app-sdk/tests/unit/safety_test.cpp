#include <gtest/gtest.h>
#include <memory>
#include "state_machines/safety_state_machine.hpp"
#include "state_machines/flight_state_machine.hpp"
#include "../mocks/mock_gps.hpp"
#include "../mocks/mock_link.hpp"

// Test fixture for SafetyStateMachine
class SafetyStateMachineTest : public ::testing::Test {
protected:
    std::shared_ptr<flightstatemachine::FlightStateMachine> m_flightStateMachine;
    safetystatemachine::SafetyStateMachine m_safetyStateMachine;

    hw_sdk_mock::MockGps m_mockGps;   // Using MockGps
    hw_sdk_mock::MockLink m_mockLink; // Using MockLink

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

    // Helper to simulate GPS signal quality changes using MockGps
    void simulateGpsSignalQuality(hw_sdk_mock::MockGps::SignalQuality quality) {
        m_mockGps.setSignalQuality(quality);

        safetystatemachine::GpsSignal gpsSignal;
        gpsSignal.quality = static_cast<drone_sdk::SignalQuality>(quality);
        m_safetyStateMachine.handleGpsSignal(gpsSignal);
    }

    // Helper to simulate Link signal quality changes using MockLink
    void simulateLinkSignalQuality(hw_sdk_mock::MockLink::SignalQuality quality) {
        m_mockLink.setSignalQuality(quality);

        safetystatemachine::LinkSignal linkSignal;
        linkSignal.quality = static_cast<drone_sdk::SignalQuality>(quality);
        m_safetyStateMachine.handleLinkSignal(linkSignal);
    }
};

// Test GPS signal transition: Healthy -> Not Healthy -> Healthy
TEST_F(SafetyStateMachineTest, GpsSignalTransitions) {
    // Start with a healthy GPS signal
    simulateGpsSignalQuality(hw_sdk_mock::MockGps::SignalQuality::GOOD);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_HEALTH);

    // Simulate loss of GPS signal
    simulateGpsSignalQuality(hw_sdk_mock::MockGps::SignalQuality::NO_SIGNAL);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_NOT_HEALTHY);

    // Restore GPS signal
    simulateGpsSignalQuality(hw_sdk_mock::MockGps::SignalQuality::EXCELLENT);
    EXPECT_EQ(m_gpsState, drone_sdk::safetyState::GPS_HEALTH);
}

// Test Link signal transition: Connected -> Disconnected -> Disconnected
TEST_F(SafetyStateMachineTest, LinkSignalTransitions) {
    // Start with a connected link signal
    simulateLinkSignalQuality(hw_sdk_mock::MockLink::SignalQuality::GOOD);
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::CONNECTED);

    // Simulate loss of Link signal
    simulateLinkSignalQuality(hw_sdk_mock::MockLink::SignalQuality::NO_SIGNAL);
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::NOT_CONNECTED);

    // Attempt to restore Link signal, should remain disconnected
    simulateLinkSignalQuality(hw_sdk_mock::MockLink::SignalQuality::FAIR);
    EXPECT_EQ(m_linkState, drone_sdk::safetyState::NOT_CONNECTED);
}
