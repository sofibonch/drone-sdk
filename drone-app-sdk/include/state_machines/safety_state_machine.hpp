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

struct GpsHealthy {};
struct GpsNotHealthy {};
struct ConnectionConnected {};
struct ConnectionDisconnected {};

// Safety_SM Struct that represents the state machine
struct Safety_SM {
    // Define states for the safety state machine

    // State machine logic (transitions, actions, etc.)
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            *state<GpsHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [this] {
                // GPS signal lost
                std::cout<<"boop"<<std::endl;
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

    explicit SafetyStateMachine()
        : m_SM()
        , m_gpsState(drone_sdk::safetyState::GPS_HEALTH)
        ,  m_linkState(drone_sdk::safetyState::CONNECTED){}

    // Public function to trigger the GPS signal event
    void handleGpsSignal(const GpsSignal& gpsSignal) {
        m_SM.process_event(gpsSignal);
        updateCurrentState();
    }

    // Public function to trigger the Link signal event
    void handleLinkSignal(const LinkSignal& linkSignal) {
        m_SM.process_event(linkSignal);
        updateCurrentState();
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

    // Current GPS and link states
    drone_sdk::safetyState m_gpsState;
    drone_sdk::safetyState m_linkState;

    // Signals for GPS and Link state changes
    StateChangeSignal m_gpsStateChangeSignal;
    StateChangeSignal m_linkStateChangeSignal;
    void updateCurrentState() {
        // Update m_currentState based on the current state of the state machine using is()
        if (m_SM.is(boost::sml::state<GpsHealthy>)) {
            m_gpsState = drone_sdk::safetyState::GPS_HEALTH;
        } else if (m_SM.is(boost::sml::state<GpsNotHealthy>)) {
            m_gpsState = drone_sdk::safetyState::GPS_NOT_HEALTHY;
        }
        if (m_SM.is(boost::sml::state<ConnectionConnected>)) {
            m_linkState = drone_sdk::safetyState::CONNECTED;
        } else if (m_SM.is(boost::sml::state<ConnectionDisconnected>)) {
            m_linkState = drone_sdk::safetyState::NOT_CONNECTED;
        }
    }

};

} // namespace safetystatemachine

#endif // SAFETY_STATE_MACHINE_HPP
