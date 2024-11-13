#ifndef SAFETY_STATE_MACHINE_HPP
#define SAFETY_STATE_MACHINE_HPP

#include <boost/sml.hpp>
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

    // The pointer to FlightStateMachine that will be used to trigger emergency land
    flightstatemachine::FlightStateMachine* flightStateMachine;

    // State machine logic (transitions, actions, etc.)
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            // Transition when GPS signal is lost (quality is NO_SIGNAL)
            *state<GpsHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [this] {
                std::cout << "GPS signal lost - Triggering Emergency Land\n";
                // Call emergency landing in the FlightStateMachine
                if (flightStateMachine) {
                    flightStateMachine->triggerSafetyViolation();
                }
            } = state<GpsNotHealthy>,

            // Transition when GPS signal is restored
            state<GpsNotHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                std::cout << "GPS signal restored\n";
            } = state<GpsHealthy>,

            // Transition when link signal is lost
            *state<ConnectionConnected> + event<LinkSignal> [([](const LinkSignal& ls) { return ls.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [this] {
                std::cout << "Link signal lost - Triggering Emergency Land\n";
                // Call emergency landing in the FlightStateMachine
                if (flightStateMachine) {
                    flightStateMachine->triggerSafetyViolation();
                }
            } = state<ConnectionDisconnected>,

            // Transition when link signal is restored
            state<ConnectionDisconnected> + event<LinkSignal> [([](const LinkSignal& ls) { return ls.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                std::cout << "Link signal restored\n";
            } = state<ConnectionConnected>
        );
    }
};

// SafetyStateMachine class encapsulates the Safety_SM state machine
class SafetyStateMachine {
public:
    // Constructor initializing the state machine and passing a pointer to FlightStateMachine
    explicit SafetyStateMachine(std::shared_ptr<flightstatemachine::FlightStateMachine> flightStateMachine)
        : m_SM(), m_flightStateMachine(flightStateMachine.get()) {
        // Set the FlightStateMachine pointer inside the state machine
       
    }

    // Public function to trigger the GPS signal event
    void handleGpsSignal(const GpsSignal& gpsSignal) {
        m_SM.process_event(gpsSignal);
    }

    // Public function to trigger the Link signal event
    void handleLinkSignal(const LinkSignal& linkSignal) {
        m_SM.process_event(linkSignal);
    }

private:
    // The state machine instance
    boost::sml::sm<Safety_SM> m_SM;
    flightstatemachine::FlightStateMachine* m_flightStateMachine;  // Pointer to the FlightStateMachine
};

} // namespace safetystatemachine

#endif // SAFETY_STATE_MACHINE_HPP
