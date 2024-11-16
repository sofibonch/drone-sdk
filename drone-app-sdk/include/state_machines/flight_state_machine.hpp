#ifndef FLIGHT_STATE_MACHINE_HPP
#define FLIGHT_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>  // For signal2
#include <iostream>
#include "icd.hpp"  // For SignalQuality and Location

namespace flightstatemachine {

// Events for the FlightStateMachine
struct TakeoffEvent {};
struct AirborneEvent {};
struct HoverEvent {};
struct TaskCompleteEvent {};
struct SafetyViolationEvent {};

// States for the FlightStateMachine
struct Landed {};
struct Takeoff {};
struct Airborne {};
struct Hover {};
struct EmergencyLand {};
struct ReturnHome {};

// Flight_SM struct represents the state machine logic and transitions
struct Flight_SM {
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            // Initial state
            *state<Landed> + event<TakeoffEvent> = state<Takeoff>,

            // Transition from Takeoff to Airborne
            state<Takeoff> + event<AirborneEvent> = state<Airborne>,

            // Transition between Airborne and Hover
            state<Airborne> + event<HoverEvent> = state<Hover>,
            state<Hover> + event<AirborneEvent> = state<Airborne>,

            // Transition from Airborne to ReturnHome
            state<Airborne> + event<TaskCompleteEvent> = state<ReturnHome>,

            // Transition from any state to Emergency Land on safety violation
            state<Landed> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<Takeoff> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<Airborne> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<Hover> + event<SafetyViolationEvent> = state<EmergencyLand>,
            state<ReturnHome> + event<SafetyViolationEvent> = state<EmergencyLand>,

            // Transition from Emergency Land to Landed (end of emergency)
            state<EmergencyLand> + event<TaskCompleteEvent> = state<Landed>,

            // Handle invalid transitions: do nothing from Emergency Land
            state<EmergencyLand> + event<AirborneEvent> =  state<EmergencyLand>,  // Invalid transition to Airborne
            state<EmergencyLand> + event<HoverEvent> = state<EmergencyLand>,      // Invalid transition to Hover
            state<EmergencyLand> + event<TakeoffEvent> = state<EmergencyLand>     // Invalid transition to Takeoff
        );
    }
};

class FlightStateMachine {
public:


    // Constructor initializing the state machine
    FlightStateMachine()
        : m_SM() 
        {}

    // Define signal to notify subscribers about state changes
    boost::signals2::signal<void(drone_sdk::FlightState)> stateChanged;

    // Public function to trigger takeoff event
    void triggerTakeoff() {
        m_SM.process_event(TakeoffEvent());
        updateCurrentState();
    }

    // Public function to trigger airborne event
    void triggerAirborne() {
        m_SM.process_event(AirborneEvent());
        updateCurrentState();
    }

    // Public function to trigger hover event
    void triggerHover() {
        m_SM.process_event(HoverEvent());
        updateCurrentState();
    }

    // Public function to trigger task completion event
    void triggerTaskComplete() {
        m_SM.process_event(TaskCompleteEvent());
        updateCurrentState();
    }

    // Public function to trigger safety violation event
    void triggerSafetyViolation() {
        m_SM.process_event(SafetyViolationEvent());
        updateCurrentState();
    }

    // Public function to process safety violation (e.g., emergency land)
    void handleEmergencyLand() {
        m_SM.process_event(SafetyViolationEvent());
        updateCurrentState();
    }

    // Function to get the current flight state
    drone_sdk::FlightState getCurrentState() const {
        return m_currentState;
    }

    // Public function to subscribe to state changes
    boost::signals2::connection subscribeToStateChange(const boost::signals2::signal<void(drone_sdk::FlightState)>::slot_type& subscriber) {
        return stateChanged.connect(subscriber);
    }

    // Handle GPS state changes from the SafetyStateMachine
    void handleGpsStateChange(drone_sdk::safetyState gpsState) {
        if (gpsState == drone_sdk::safetyState::GPS_NOT_HEALTHY) {
            //std::cout << "FlightSM: GPS Not Healthy - Triggering Safety Violation" << std::endl;
            triggerSafetyViolation();
        }
    }

    // Handle Link state changes from the SafetyStateMachine
    void handleLinkStateChange(drone_sdk::safetyState linkState) {
        if (linkState == drone_sdk::safetyState::NOT_CONNECTED) {
           // std::cout << "FlightSM: Link Disconnected - Triggering Safety Violation" << std::endl;
            triggerSafetyViolation();
        }
    }

private:
//add a private function that handles the change based on the flghit state machine
    void updateCurrentState() {
        // Update m_currentState based on the current state of the state machine using is()
        if (m_SM.is(boost::sml::state<Landed>)) {
            m_currentState = drone_sdk::FlightState::LANDED;
        } else if (m_SM.is(boost::sml::state<Takeoff>)) {
            m_currentState = drone_sdk::FlightState::TAKEOFF;
        } else if (m_SM.is(boost::sml::state<Airborne>)) {
            m_currentState = drone_sdk::FlightState::AIRBORNE;
        } else if (m_SM.is(boost::sml::state<Hover>)) {
            m_currentState = drone_sdk::FlightState::HOVER;
        } else if (m_SM.is(boost::sml::state<ReturnHome>)) {
            m_currentState = drone_sdk::FlightState::RETURN_HOME;
        } else if (m_SM.is(boost::sml::state<EmergencyLand>)) {
            m_currentState = drone_sdk::FlightState::EMERGENCY_LAND;
        }
    }
    //
private:
    boost::sml::sm<Flight_SM> m_SM;
    drone_sdk::FlightState m_currentState = drone_sdk::FlightState::LANDED;
   // drone_sdk::Location m_cur_loc {}


};

} // namespace flightstatemachine

#endif // FLIGHT_STATE_MACHINE_HPP
