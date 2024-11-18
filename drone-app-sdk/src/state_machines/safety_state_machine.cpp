#include "state_machines/safety_state_machine.hpp"

namespace safetystatemachine
{
    SafetyStateMachine::SafetyStateMachine()
        : m_SM(),
          m_gpsState(drone_sdk::safetyState::GPS_HEALTH),
          m_linkState(drone_sdk::safetyState::CONNECTED) {}

    // Handle GPS signal events
    void SafetyStateMachine::handleGpsSignal(const drone_sdk::SignalQuality &gpsSignal)
    {
        drone_sdk::safetyState prevGpsState = m_gpsState;
        GpsSignal signal(gpsSignal);
        m_SM.process_event(signal);
        updateCurrentState();
        if (m_gpsState != prevGpsState)
        {
            m_gpsStateChangeSignal(m_gpsState);
        }
    }

    // Handle Link signal events
    void SafetyStateMachine::handleLinkSignal(const drone_sdk::SignalQuality &linkSignal)
    {
        drone_sdk::safetyState prevLinkState = m_linkState;
        LinkSignal signal(linkSignal);
        m_SM.process_event(signal);
        updateCurrentState();
        if (m_linkState != prevLinkState)
        {
            m_linkStateChangeSignal(m_linkState);
        }
    }

    // Subscribe to GPS state changes
    boost::signals2::connection SafetyStateMachine::subscribeToGpsState(const StateChangeSignal::slot_type &subscriber)
    {
        return m_gpsStateChangeSignal.connect(subscriber);
    }

    // Subscribe to Link state changes
    boost::signals2::connection SafetyStateMachine::subscribeToLinkState(const StateChangeSignal::slot_type &subscriber)
    {
        return m_linkStateChangeSignal.connect(subscriber);
    }

    // Get current GPS state
    drone_sdk::safetyState SafetyStateMachine::getCurrentGpsState() const
    {
        return m_gpsState;
    }

    // Get current Link state
    drone_sdk::safetyState SafetyStateMachine::getCurrentLinkState() const
    {
        return m_linkState;
    }

    // Update current states and notify if changes occur
    void SafetyStateMachine::updateCurrentState()
    {

        if (m_SM.is(boost::sml::state<GpsHealthy>))
        {
            m_gpsState = drone_sdk::safetyState::GPS_HEALTH;
        }
        else if (m_SM.is(boost::sml::state<GpsNotHealthy>))
        {
            m_gpsState = drone_sdk::safetyState::GPS_NOT_HEALTHY;
        }

        if (m_SM.is(boost::sml::state<ConnectionConnected>))
        {
            m_linkState = drone_sdk::safetyState::CONNECTED;
        }
        else if (m_SM.is(boost::sml::state<ConnectionDisconnected>))
        {
            m_linkState = drone_sdk::safetyState::NOT_CONNECTED;
        }
    }

} // namespace safetystatemachine
