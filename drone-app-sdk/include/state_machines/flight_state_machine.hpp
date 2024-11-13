#ifndef FLIGHT_STATE_MACHINE_HPP
#define FLIGHT_STATE_MACHINE_HPP

#include <boost/sml.hpp>
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
            *state<Landed> + event<TakeoffEvent> / [] {
                std::cout << "Drone taking off...\n";
            } = state<Takeoff>,

            // Transition from Takeoff to Airborne
            state<Takeoff> + event<AirborneEvent> / [] {
                std::cout << "Drone is airborne...\n";
            } = state<Airborne>,

            // Transition between Airborne and Hover
            state<Airborne> + event<HoverEvent> / [] {
                std::cout << "Drone is hovering...\n";
            } = state<Hover>,
            state<Hover> + event<AirborneEvent> / [] {
                std::cout << "Drone is airborne...\n";
            } = state<Airborne>,

            // Transition from Airborne to ReturnHome
            state<Airborne> + event<TaskCompleteEvent> / [] {
                std::cout << "Task complete, returning home...\n";
            } = state<ReturnHome>,

            // Transition from any state to Emergency Land on safety violation
            state<Landed> + event<SafetyViolationEvent> / [] {
                std::cout << "Emergency land triggered!\n";
            } = state<EmergencyLand>,
            state<Takeoff> + event<SafetyViolationEvent> / [] {
                std::cout << "Emergency land triggered!\n";
            } = state<EmergencyLand>,
            state<Airborne> + event<SafetyViolationEvent> / [] {
                std::cout << "Emergency land triggered!\n";
            } = state<EmergencyLand>,
            state<Hover> + event<SafetyViolationEvent> / [] {
                std::cout << "Emergency land triggered!\n";
            } = state<EmergencyLand>,
            state<ReturnHome> + event<SafetyViolationEvent> / [] {
                std::cout << "Emergency land triggered!\n";
            } = state<EmergencyLand>,

            // Transition from Emergency Land to Landed (end of emergency)
            state<EmergencyLand> + event<TaskCompleteEvent> / [] {
                std::cout << "Emergency landing complete, drone is now landed.\n";
            } = state<Landed>
        );
    }
};

// FlightStateMachine class that encapsulates the Flight_SM state machine
class FlightStateMachine {
public:
    // Constructor initializing the state machine
    FlightStateMachine()
        : m_SM() {}

    // Public function to trigger takeoff event
    void triggerTakeoff() {
        m_SM.process_event(TakeoffEvent());
    }

    // Public function to trigger airborne event
    void triggerAirborne() {
        m_SM.process_event(AirborneEvent());
    }

    // Public function to trigger hover event
    void triggerHover() {
        m_SM.process_event(HoverEvent());
    }

    // Public function to trigger task completion event
    void triggerTaskComplete() {
        m_SM.process_event(TaskCompleteEvent());
    }

    // Public function to trigger safety violation event
    void triggerSafetyViolation() {
        m_SM.process_event(SafetyViolationEvent());
    }

    // Public function to process safety violation (e.g., emergency land)
    void handleEmergencyLand() {
        m_SM.process_event(SafetyViolationEvent());
    }

private:
    // The state machine instance
    boost::sml::sm<Flight_SM> m_SM;
};

} // namespace flightstatemachine

#endif // FLIGHT_STATE_MACHINE_HPP
