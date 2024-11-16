#include "state_machines/command_state_machine.hpp"

namespace commandstatemachine {

using namespace boost::sml;

// CommandStateMachine constructor
CommandStateMachine::CommandStateMachine()
    : m_SM(), m_currentState(drone_sdk::CommandStatus::IDLE), m_currMission(drone_sdk::CurrentMission::HOVER) {}

// Handle task assignment
void CommandStateMachine::handleTaskAssigned(
    drone_sdk::CurrentMission newMission,
    const std::optional<drone_sdk::Location>& singleDestination,
    const std::optional<std::queue<drone_sdk::Location>>& pathDestinations) {
    
    if (m_currentState == drone_sdk::CommandStatus::BUSY) {
        handleTaskAbortedOverride();
    }

    m_SM.process_event(TaskAssigned{});
    updateCurrentMission(newMission);
    updateCurrentState();

    if (singleDestination && pathDestinations) {
        throw std::invalid_argument(
            "Both singleDestination and pathDestinations cannot be provided simultaneously");
    }

    switch (newMission) {
    case drone_sdk::CurrentMission::GOTO:
        if (!singleDestination) {
            throw std::invalid_argument("GOTO mission requires a single destination");
        }
        m_destination = *singleDestination;
        break;

    case drone_sdk::CurrentMission::HOME:
        m_destination = m_home;
        break;

    case drone_sdk::CurrentMission::HOVER:
        break;

    case drone_sdk::CurrentMission::PATH:
        if (!pathDestinations) {
            throw std::invalid_argument("PATH mission requires a queue of destinations");
        }
        m_pathQueue = *pathDestinations;
        m_destination = m_pathQueue.front();
        m_pathQueue.pop();
        break;

    default:
        throw std::invalid_argument("Unknown mission type");
    }
}

// Handle task aborted due to safety
void CommandStateMachine::handleTaskAbortedSafety() {
    m_SM.process_event(TaskAborted{});
    m_destination = m_home;
    updateCurrentState();
}

// Subscribe to state changes
boost::signals2::connection CommandStateMachine::subscribeToState(const StateChangeSignal::slot_type& subscriber) {
    return m_stateChangeSignal.connect(subscriber);
}

// Subscribe to path waypoint updates
boost::signals2::connection CommandStateMachine::subscribeToPathWaypoint(const PathWayPoint::slot_type& subscriber) {
    return m_pathWaypoint.connect(subscriber);
}

// Get the current state
drone_sdk::CommandStatus CommandStateMachine::getCurrentState() const {
    return m_currentState;
}

// Handle GPS location updates
void CommandStateMachine::handleGpsLocationUpdate(const drone_sdk::Location& newLocation) {
    m_currLocation = newLocation;
    if (m_currLocation == m_destination) {
        switch (m_currMission) {
        case drone_sdk::CurrentMission::GOTO:
        case drone_sdk::CurrentMission::HOME:
            handleTaskCompleted();
            break;
        case drone_sdk::CurrentMission::PATH:
            handleTaskPathUpdate();
            break;
        case drone_sdk::CurrentMission::HOVER:
            // Remain in the current state
            break;

        default:
            break;
        }
    }
}

// Set home base
void CommandStateMachine::setHomebase(const drone_sdk::Location& newHome) {
    m_home = newHome;
}

// Handle task aborted override
void CommandStateMachine::handleTaskAbortedOverride() {
    m_SM.process_event(TaskAborted{});
    updateCurrentState();
    m_SM.process_event(TaskCompleted{});
    updateCurrentState();
}

// Handle task completion
void CommandStateMachine::handleTaskCompleted() {
    m_SM.process_event(TaskCompleted{});
    updateCurrentState();
}

// Handle path updates for PATH mission
void CommandStateMachine::handleTaskPathUpdate() {
    if (m_pathQueue.empty()) {
        handleTaskCompleted();
    } else {
        m_pathWaypoint(m_destination);
        m_destination = m_pathQueue.front();
        m_pathQueue.pop();
    }
}

// Update current state
void CommandStateMachine::updateCurrentState() {
    drone_sdk::CommandStatus prevState = m_currentState;

    if (m_SM.is(state<Idle>)) {
        m_currentState = drone_sdk::CommandStatus::IDLE;
    } else if (m_SM.is(state<Busy>)) {
        m_currentState = drone_sdk::CommandStatus::BUSY;
    } else if (m_SM.is(state<MissionComplete>)) {
        m_currentState = drone_sdk::CommandStatus::MISSION_COMPLETE;
    } else if (m_SM.is(state<MissionAbort>)) {
        m_currentState = drone_sdk::CommandStatus::MISSION_ABORT;
    }

    if (m_currentState != prevState) {
        m_stateChangeSignal(m_currentState);
    }
}

// Update current mission
void CommandStateMachine::updateCurrentMission(drone_sdk::CurrentMission newMission) {
    m_currMission = newMission;
}

} // namespace commandstatemachine
