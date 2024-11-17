#ifndef FLIGHT_STATE_MACHINE_HPP
#define FLIGHT_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>  // For signal2
#include <iostream>
#include "icd.hpp"  // For SignalQuality and Location

namespace flightstatemachine {

/**
 * @brief Events for the FlightStateMachine
 * These events trigger transitions between states in the state machine.
 */
struct TakeoffEvent {};      ///< Triggered to initiate takeoff.
struct AirborneEvent {};     ///< Triggered when the drone becomes airborne.
struct HoverEvent {};        ///< Triggered to transition to hover mode.
struct TaskCompleteEvent {}; ///< Triggered when the drone completes its task.
struct SafetyViolationEvent {}; ///< Triggered during a safety violation.

/**
 * @brief States for the FlightStateMachine
 * These represent the possible states the drone can be in.
 */
struct Landed {};         ///< The drone is on the ground.
struct Takeoff {};        ///< The drone is taking off.
struct Airborne {};       ///< The drone is flying normally.
struct Hover {};          ///< The drone is hovering in place.
struct EmergencyLand {};  ///< The drone is performing an emergency landing.
struct ReturnHome {};     ///< The drone is returning to its home location.

/**
 * @brief Flight_SM represents the state machine logic and transitions.
 * 
 * @details This struct defines the transitions between states using Boost.SML.
 *          Transitions include valid moves between states and invalid scenarios.
 */
struct Flight_SM {
    /**
     * @brief Defines the state machine transitions.
     * @return The transition table for the state machine.
     */
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            *state<Landed> + event<TakeoffEvent> = state<Takeoff>,
            state<Takeoff> + event<AirborneEvent> = state<Airborne>,
            state<Airborne> + event<HoverEvent> = state<Hover>,
            state<Hover> + event<AirborneEvent> = state<Airborne>,
            state<Airborne> + event<TaskCompleteEvent> = state<ReturnHome>,
            state<Landed> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<Takeoff> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<Airborne> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<Hover> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<ReturnHome> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<EmergencyLand> + event<TaskCompleteEvent> = state<Landed>,
            state<EmergencyLand> + event<AirborneEvent> = state<EmergencyLand>,
            state<EmergencyLand> + event<HoverEvent> = state<EmergencyLand>,
            state<EmergencyLand> + event<TakeoffEvent> = state<EmergencyLand>
        );
    }
};

/**
 * @brief FlightStateMachine class manages the state transitions and signals.
 */
class FlightStateMachine {
public:
    /**
     * @brief Constructor for the FlightStateMachine.
     */
    FlightStateMachine();
    ~FlightStateMachine()=default;
    /**
     * @brief Trigger a takeoff event.
     */
    void triggerTakeoff();

    /**
     * @brief Trigger an airborne event.
     */
    void triggerAirborne();

    /**
     * @brief Trigger a hover event.
     */
    void triggerHover();

    /**
     * @brief Notify the state machine that a task has been completed.
     */
    void triggerTaskComplete();

    /**
     * @brief Notify the state machine of a safety violation.
     */
    void triggerSafetyViolation();

    /**
     * @brief Handle emergency landing procedures.
     */
    void handleEmergencyLand();

    /**
     * @brief Handle changes in GPS state.
     * @param gpsState The current state of the GPS module.
     */
    void handleGpsStateChange(drone_sdk::safetyState gpsState);

    /**
     * @brief Handle changes in Link state.
     * @param linkState The current state of the communication link.
     */
    void handleLinkStateChange(drone_sdk::safetyState linkState);

    /**
     * @brief Get the current flight state of the drone.
     * @return The current flight state as a FlightState enum.
     */
    drone_sdk::FlightState getCurrentState() const;

    /**
     * @brief Subscribe to flight state changes.
     * @param subscriber A callback function to be triggered on state change.
     * @return A connection object for managing the subscription.
     */
    boost::signals2::connection subscribeToStateChange(
        const boost::signals2::signal<void(drone_sdk::FlightState)>::slot_type& subscriber);

private:
    /**
     * @brief Update the current flight state and notify subscribers.
     */
    void updateCurrentState();

    boost::sml::sm<Flight_SM> m_SM; ///< The state machine instance.
    drone_sdk::FlightState m_currentState = drone_sdk::FlightState::LANDED; ///< Current state of the drone.

    // Signal for notifying subscribers of state changes.
    boost::signals2::signal<void(drone_sdk::FlightState)> stateChanged;
};

} // namespace flightstatemachine

#endif // FLIGHT_STATE_MACHINE_HPP
