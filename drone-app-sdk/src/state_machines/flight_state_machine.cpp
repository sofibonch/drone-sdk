#include "state_machines/flight_state_machine.hpp"

namespace flightstatemachine
{

    // FlightStateMachine class implementation
    FlightStateMachine::FlightStateMachine()
        : m_SM() {}

    void FlightStateMachine::triggerTakeoff()
    {
        m_SM.process_event(TakeoffEvent());
        updateCurrentState();
    }

    void FlightStateMachine::triggerAirborne()
    {
        if (m_currentState == drone_sdk::FlightState::LANDED)
        {
            triggerTakeoff();
        }
        else
        {
            m_SM.process_event(AirborneEvent());
        }
        updateCurrentState();
    }

    void FlightStateMachine::triggerHover()
    {
        if (m_currentState == drone_sdk::FlightState::LANDED)
        {
            triggerTakeoff();
        }
        m_SM.process_event(HoverEvent());
        updateCurrentState();
    }

    void FlightStateMachine::triggerTaskComplete()
    {
        m_SM.process_event(TaskCompleteEvent());
        updateCurrentState();
    }
    void FlightStateMachine::triggerReturnHome()
    {
        m_SM.process_event(ReturnHomeEvent());
        updateCurrentState();
    }
    void FlightStateMachine::triggerSafetyViolation()
    {
        m_SM.process_event(SafetyViolationEvent());
        updateCurrentState();
    }

    void FlightStateMachine::handleEmergencyLand()
    {
        m_SM.process_event(SafetyViolationEvent());
        updateCurrentState();
    }

    void FlightStateMachine::triggerLand()
    {
        m_SM.process_event(LandEvent());
        updateCurrentState();
    }

    void FlightStateMachine::handleCommandStateChange(drone_sdk::CommandStatus commadState)
    {
        switch (commadState)
        {
        // change state to idle, task is completed- stay in hover position
        case drone_sdk::CommandStatus::IDLE:
            triggerHover();
            break;
        case drone_sdk::CommandStatus::BUSY:
            triggerAirborne();
            break;
        case drone_sdk::CommandStatus::MISSION_ABORT:
            triggerSafetyViolation();
        default:
            break;
        }
    }
    void FlightStateMachine::handleNewMission(drone_sdk::CurrentMission newMission)
    {
        switch (newMission)
        {
        case drone_sdk::CurrentMission::LANDED:
            triggerLand();
            break;
        case drone_sdk::CurrentMission::GOTO:
            triggerAirborne();
            break;
        case drone_sdk::CurrentMission::PATH:
            triggerAirborne();
            break;
        case drone_sdk::CurrentMission::HOVER:
            triggerHover();
            break;
        case drone_sdk::CurrentMission::HOME:
            triggerReturnHome();
            break;
        case drone_sdk::CurrentMission::EMERGENCY:
            triggerSafetyViolation();
            break;
        default:
            triggerSafetyViolation();
            break;
        }
    }

    drone_sdk::FlightState FlightStateMachine::getCurrentState() const
    {
        return m_currentState;
    }

    void FlightStateMachine::updateCurrentState()
    {
        if (m_SM.is(boost::sml::state<Landed>))
        {
            m_currentState = drone_sdk::FlightState::LANDED;
        }
        else if (m_SM.is(boost::sml::state<Takeoff>))
        {
            m_currentState = drone_sdk::FlightState::TAKEOFF;
        }
        else if (m_SM.is(boost::sml::state<Airborne>))
        {
            m_currentState = drone_sdk::FlightState::AIRBORNE;
        }
        else if (m_SM.is(boost::sml::state<Hover>))
        {
            m_currentState = drone_sdk::FlightState::HOVER;
        }
        else if (m_SM.is(boost::sml::state<ReturnHome>))
        {
            m_currentState = drone_sdk::FlightState::RETURN_HOME;
        }
        else if (m_SM.is(boost::sml::state<EmergencyLand>))
        {
            m_currentState = drone_sdk::FlightState::EMERGENCY_LAND;
        }

        m_stateChangedSignal(m_currentState);
    }

} // namespace flightstatemachine
