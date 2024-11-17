#include "state_machines/flight_state_machine.hpp"

namespace flightstatemachine {

// FlightStateMachine class implementation
FlightStateMachine::FlightStateMachine()
    : m_SM() {}

void FlightStateMachine::triggerTakeoff() {
    m_SM.process_event(TakeoffEvent());
    updateCurrentState();
}

void FlightStateMachine::triggerAirborne() {
    m_SM.process_event(AirborneEvent());
    updateCurrentState();
}

void FlightStateMachine::triggerHover() {
    m_SM.process_event(HoverEvent());
    updateCurrentState();
}

void FlightStateMachine::triggerTaskComplete() {
    m_SM.process_event(TaskCompleteEvent());
    updateCurrentState();
}

void FlightStateMachine::triggerSafetyViolation() {
    m_SM.process_event(SafetyViolationEvent());
    updateCurrentState();
}

void FlightStateMachine::handleEmergencyLand() {
    m_SM.process_event(SafetyViolationEvent());
    updateCurrentState();
}

void FlightStateMachine::handleGpsStateChange(drone_sdk::safetyState gpsState) {
    if (gpsState == drone_sdk::safetyState::GPS_NOT_HEALTHY) {
        triggerSafetyViolation();
    }
}

void FlightStateMachine::handleLinkStateChange(drone_sdk::safetyState linkState) {
    if (linkState == drone_sdk::safetyState::NOT_CONNECTED) {
        triggerSafetyViolation();
    }
}

drone_sdk::FlightState FlightStateMachine::getCurrentState() const {
    return m_currentState;
}

boost::signals2::connection FlightStateMachine::subscribeToStateChange(
    const boost::signals2::signal<void(drone_sdk::FlightState)>::slot_type& subscriber) {
    return stateChanged.connect(subscriber);
}

void FlightStateMachine::updateCurrentState() {
    drone_sdk::FlightState prev_stat = m_currentState;

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

    if (m_currentState != prev_stat) {
        stateChanged(m_currentState);
    }
}

} // namespace flightstatemachine
