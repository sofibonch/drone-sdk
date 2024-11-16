#ifndef COMMAND_STATE_MACHINE_HPP
#define COMMAND_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>
#include <queue>
#include <optional>
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
            state<MissionComplete> + event<TaskCompleted> = state<Idle>,    // MissionComplete → Idle for next assignment
            state<MissionAbort> + event<TaskCompleted> = state<Idle>        // MissionAbort → Idle for next assignment
        );
    }
};

// CommandStateMachine class encapsulates the Command_SM state machine
//home is set to default (0,0,0)
class CommandStateMachine {
public:
    using StateChangeSignal = boost::signals2::signal<void(drone_sdk::CommandStatus)>;
    using PathWayPoint = boost::signals2::signal<void(drone_sdk::Location)>;

    CommandStateMachine();

    void handleTaskAssigned(
        drone_sdk::CurrentMission newMission,
        const std::optional<drone_sdk::Location>& singleDestination = std::nullopt,
        const std::optional<std::queue<drone_sdk::Location>>& pathDestinations = std::nullopt);

    void handleTaskAbortedSafety();
    void handleGpsLocationUpdate(const drone_sdk::Location& newLocation);

    void setHomebase(const drone_sdk::Location& newHome);
    drone_sdk::Location getHomebase(){

        return m_home;
    }

    boost::signals2::connection subscribeToState(const StateChangeSignal::slot_type& subscriber);
    boost::signals2::connection subscribeToPathWaypoint(const PathWayPoint::slot_type& subscriber);

    drone_sdk::CommandStatus getCurrentState() const;

private:
    void handleTaskAbortedOverride();
    void handleTaskCompleted();
    void handleTaskPathUpdate();
    void updateCurrentState();
    void updateCurrentMission(drone_sdk::CurrentMission newMission);

private:
    boost::sml::sm<Command_SM> m_SM;
    drone_sdk::CommandStatus m_currentState;
    StateChangeSignal m_stateChangeSignal;
    PathWayPoint m_pathWaypoint;

    drone_sdk::CurrentMission m_currMission;
    drone_sdk::Location m_currLocation;
    drone_sdk::Location m_destination;
    drone_sdk::Location m_home;
    std::queue<drone_sdk::Location> m_pathQueue;
};

} // namespace commandstatemachine
#endif // COMMAND_STATE_MACHINE_HPP
