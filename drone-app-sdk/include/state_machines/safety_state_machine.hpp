#ifndef SAFETY_STATE_MACHINE_HPP
#define SAFETY_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>
#include <iostream>

#include "gps/gps.hpp"
#include "link/link.hpp"
#include "icd.hpp"

namespace safetystatemachine {

/**
 * @brief Event representing the current GPS signal quality.
 */
struct GpsSignal {
    drone_sdk::SignalQuality quality; /**< Quality of the GPS signal. */
};

/**
 * @brief Event representing the current link signal quality.
 */
struct LinkSignal {
    drone_sdk::SignalQuality quality; /**< Quality of the link signal. */
};

/**
 * @brief State representing a healthy GPS signal.
 */
struct GpsHealthy {};

/**
 * @brief State representing a loss of GPS signal.
 */
struct GpsNotHealthy {};

/**
 * @brief State representing a connected communication link.
 */
struct ConnectionConnected {};

/**
 * @brief State representing a disconnected communication link.
 */
struct ConnectionDisconnected {};

/**
 * @brief Safety state machine logic and transitions.
 */
struct Safety_SM {
    /**
     * @brief Defines the transitions and actions of the safety state machine.
     * @return The state machine transition table.
     */
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            // GPS signal transitions
            *state<GpsHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] = state<GpsNotHealthy>,
            state<GpsNotHealthy> + event<GpsSignal> [([](const GpsSignal& gs) { return gs.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] = state<GpsHealthy>,

            // Link signal transitions
            *state<ConnectionConnected> + event<LinkSignal> [([](const LinkSignal& ls) { return ls.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] = state<ConnectionDisconnected>,
            state<ConnectionDisconnected> + event<LinkSignal> [([](const LinkSignal& ls) { return ls.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] = state<ConnectionConnected>
        );
    }
};

/**
 * @brief Class for managing the safety state machine.
 */
class SafetyStateMachine {
public:
    using StateChangeSignal = boost::signals2::signal<void(drone_sdk::safetyState)>; /**< Signal for state changes. */

    /**
     * @brief Constructor for the SafetyStateMachine.
     */
    SafetyStateMachine();

    /**
     * @brief Handle GPS signal events.
     * @param gpsSignal The current GPS signal quality.
     */
    void handleGpsSignal(const GpsSignal& gpsSignal);

    /**
     * @brief Handle Link signal events.
     * @param linkSignal The current link signal quality.
     */
    void handleLinkSignal(const LinkSignal& linkSignal);

    /**
     * @brief Subscribe to GPS state changes.
     * @param subscriber A callback function to be triggered on GPS state changes.
     * @return A connection object for managing the subscription.
     */
    boost::signals2::connection subscribeToGpsState(const StateChangeSignal::slot_type& subscriber);

    /**
     * @brief Subscribe to Link state changes.
     * @param subscriber A callback function to be triggered on Link state changes.
     * @return A connection object for managing the subscription.
     */
    boost::signals2::connection subscribeToLinkState(const StateChangeSignal::slot_type& subscriber);

    /**
     * @brief Get the current GPS state.
     * @return The current GPS state as a drone_sdk::safetyState.
     */
    drone_sdk::safetyState getCurrentGpsState() const;

    /**
     * @brief Get the current Link state.
     * @return The current Link state as a drone_sdk::safetyState.
     */
    drone_sdk::safetyState getCurrentLinkState() const;

private:
    /**
     * @brief Update the current states and notify subscribers if changes occur.
     */
    void updateCurrentState();

private:
    boost::sml::sm<Safety_SM> m_SM; /**< The underlying state machine object. */
    drone_sdk::safetyState m_gpsState; /**< Current GPS state. */
    drone_sdk::safetyState m_linkState; /**< Current Link state. */

    StateChangeSignal m_gpsStateChangeSignal; /**< Signal for GPS state changes. */
    StateChangeSignal m_linkStateChangeSignal; /**< Signal for Link state changes. */
};

} // namespace safetystatemachine

#endif // SAFETY_STATE_MACHINE_HPP
