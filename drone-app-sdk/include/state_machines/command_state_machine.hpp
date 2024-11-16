#ifndef COMMAND_STATE_MACHINE_HPP
#define COMMAND_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>
#include "icd.hpp"

namespace commandstatemachine {

// Events
struct TaskAssigned {};
struct TaskCompleted {};
struct TaskAborted {};

// States
struct Idle {};
struct Busy {};
struct MissionComplete {};
struct MissionAbort {};

// Command_SM Struct that represents the state machine
struct Command_SM {
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            *state<Idle> + event<TaskAssigned> = state<Busy>,               // Idle → Busy on task assignment
            state<Busy> + event<TaskCompleted> = state<MissionComplete>,    // Busy → MissionComplete on task completion
            state<Busy> + event<TaskAborted> = state<MissionAbort>,         // Busy → MissionAbort on task aborted
            state<MissionComplete> + event<TaskAssigned> = state<Idle>,     // MissionComplete → Idle for next assignment
            state<MissionAbort> + event<TaskAssigned> = state<Idle>         // MissionAbort → Idle for next assignment
        );
    }
};

// CommandStateMachine class encapsulates the Command_SM state machine
class CommandStateMachine {
public:
    using StateChangeSignal = boost::signals2::signal<void(drone_sdk::CommandStatus)>;

    // Constructor initializing the state machine
    CommandStateMachine() : m_SM(), m_currentState(drone_sdk::CommandStatus::IDLE) {}

    // Public function to trigger the TaskAssigned event
    void handleTaskAssigned() {
        updateCurrentState();
        m_SM.process_event(TaskAssigned{});
    }

    // Public function to trigger the TaskCompleted event
    void handleTaskCompleted() {
        updateCurrentState();
        m_SM.process_event(TaskCompleted{});
    }

    // Public function to trigger the TaskAborted event
    void handleTaskAborted() {
        updateCurrentState();
        m_SM.process_event(TaskAborted{});
    }

    // Public function to subscribe to state changes
    boost::signals2::connection subscribeToState(const StateChangeSignal::slot_type& subscriber) {
        return m_stateChangeSignal.connect(subscriber);
    }

    // Public function to get the current state
    drone_sdk::CommandStatus getCurrentState() const {
        return m_currentState;
    }

private: 
    // Function to update the current state based on the state machine
    void updateCurrentState() {
        if (m_SM.is(boost::sml::state<Idle>)) {
            m_currentState = drone_sdk::CommandStatus::IDLE;
        } else if (m_SM.is(boost::sml::state<Busy>)) {
            m_currentState = drone_sdk::CommandStatus::BUSY;
        } else if (m_SM.is(boost::sml::state<MissionComplete>)) {
            m_currentState = drone_sdk::CommandStatus::MISSION_COMPLETE;
        } else if (m_SM.is(boost::sml::state<MissionAbort>)) {
            m_currentState = drone_sdk::CommandStatus::MISSION_ABORT;
        }

        // Emit signal for state change
        m_stateChangeSignal(m_currentState);
    }

private:
    
    boost::sml::sm<Command_SM> m_SM;

    drone_sdk::CommandStatus m_currentState;
    StateChangeSignal m_stateChangeSignal;
    


};

} // namespace commandstatemachine

#endif // COMMAND_STATE_MACHINE_HPP
