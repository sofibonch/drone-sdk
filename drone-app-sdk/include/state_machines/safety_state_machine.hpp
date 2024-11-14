#ifndef SAFETY_STATE_MACHINE_HPP
#define SAFETY_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>
#include <iostream>
#include "flight_state_machine.hpp"  // Include the FlightStateMachine header
#include "gps/gps.hpp"
#include "link/link.hpp"
#include "icd.hpp"

namespace safetystatemachine {

// Events
struct GpsSignal {
    drone_sdk::SignalQuality quality;
};

struct LinkSignal {
    drone_sdk::SignalQuality quality;
};

// Safety_SM Struct that represents the state machine
struct Safety_SM {
    // Define states for the safety state machine
    struct GpsHealthy {};
    struct GpsNotHealthy {};
    struct ConnectionConnected {};
    struct ConnectionDisconnected {};

    // State machine logic (transitions, actions, etc.)
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            *state<GpsHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [this] {
                // GPS signal lost
            } = state<GpsNotHealthy>,

            state<GpsNotHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                // GPS signal restored
            } = state<GpsHealthy>,

            *state<ConnectionConnected> + event<LinkSignal> [([](const LinkSignal& ls) { return ls.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [this] {
                // Link signal lost
            } = state<ConnectionDisconnected>,

            state<ConnectionDisconnected> + event<LinkSignal> [([](const LinkSignal& ls) { return ls.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                // Link signal restored
            } = state<ConnectionConnected>
        );
    }
};

// SafetyStateMachine class encapsulates the Safety_SM state machine
class SafetyStateMachine {
public:
    using StateChangeSignal = boost::signals2::signal<void(drone_sdk::safetyState)>;

    // Constructor initializing the state machine and passing a pointer to FlightStateMachine
    explicit SafetyStateMachine(std::shared_ptr<flightstatemachine::FlightStateMachine> flightStateMachine)
        : m_SM(), m_flightStateMachine(flightStateMachine.get()),
          m_gpsState(drone_sdk::safetyState::GPS_HEALTH),
          m_linkState(drone_sdk::safetyState::CONNECTED),
          m_linkDisconnectedLock(false) {}

    // Public function to trigger the GPS signal event
    void handleGpsSignal(const GpsSignal& gpsSignal) {
        m_SM.process_event(gpsSignal);
        updateGpsState(gpsSignal);
    }

    // Public function to trigger the Link signal event
    void handleLinkSignal(const LinkSignal& linkSignal) {
        if (!m_linkDisconnectedLock) {  // Only process if the lock is not set
            m_SM.process_event(linkSignal);
            updateLinkState(linkSignal);
        }
    }

    // Public function to subscribe to GPS state changes
    boost::signals2::connection subscribeToGpsState(const StateChangeSignal::slot_type& subscriber) {
        return m_gpsStateChangeSignal.connect(subscriber);
    }

    // Public function to subscribe to Link state changes
    boost::signals2::connection subscribeToLinkState(const StateChangeSignal::slot_type& subscriber) {
        return m_linkStateChangeSignal.connect(subscriber);
    }

    // Public function to get the current GPS state
    drone_sdk::safetyState getCurrentGpsState() const {
        return m_gpsState;
    }

    // Public function to get the current link state
    drone_sdk::safetyState getCurrentLinkState() const {
        return m_linkState;
    }

private:
    // The state machine instance
    boost::sml::sm<Safety_SM> m_SM;
    flightstatemachine::FlightStateMachine* m_flightStateMachine;  // Pointer to the FlightStateMachine

    // Current GPS and link states
    drone_sdk::safetyState m_gpsState;
    drone_sdk::safetyState m_linkState;

    // Signals for GPS and Link state changes
    StateChangeSignal m_gpsStateChangeSignal;
    StateChangeSignal m_linkStateChangeSignal;

    // Lock to prevent reconnection if link is disconnected
    bool m_linkDisconnectedLock;

    // Private function to update the GPS state based on the signal
    void updateGpsState(const GpsSignal& gpsSignal) {
        auto newState = (gpsSignal.quality == drone_sdk::SignalQuality::NO_SIGNAL) 
                     ? drone_sdk::safetyState::GPS_NOT_HEALTHY 
                     : drone_sdk::safetyState::GPS_HEALTH;
        if (m_gpsState != newState) {
            m_gpsState = newState;
            m_gpsStateChangeSignal(m_gpsState);  // Notify subscribers of GPS state change
        }
    }

    // Private function to update the Link state based on the signal
    void updateLinkState(const LinkSignal& linkSignal) {
        auto newState = (linkSignal.quality == drone_sdk::SignalQuality::NO_SIGNAL) 
                      ? drone_sdk::safetyState::NOT_CONNECTED 
                      : drone_sdk::safetyState::CONNECTED;

        if (m_linkState != newState) {
            if (newState == drone_sdk::safetyState::NOT_CONNECTED) {
                m_linkDisconnectedLock = true;  // Lock the link to prevent reconnection
            }
            m_linkState = newState;
            m_linkStateChangeSignal(m_linkState);  // Notify subscribers of Link state change
        }
    }
};

} // namespace safetystatemachine

#endif // SAFETY_STATE_MACHINE_HPP
