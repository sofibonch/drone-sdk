#ifndef COMMAND_STATE_MACHINE_HPP
#define COMMAND_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <boost/signals2.hpp>
#include <queue>
#include <optional>
#include "icd.hpp"
#include <iostream>
namespace commandstatemachine
{

    // Events represent triggers for state transitions in the state machine
    struct TaskAssigned
    {
    }; ///< Event triggered when a new task is assigned.
    struct TaskCompleted
    {
    }; ///< Event triggered when a task is completed.
    struct TaskAborted
    {
    }; ///< Event triggered when a task is aborted because of emergency.

    // States define the possible states of the command state machine
    struct Idle
    {
    }; ///< State representing no active task (idle).
    struct Busy
    {
    }; ///< State representing an ongoing task.
    struct MissionAbort
    {
    }; ///< State representing an aborted mission because safety reasons.

    // Command_SM struct defines the state machine transitions and logic
    struct Command_SM
    {
        /**
         * @brief Defines the transition table for the state machine.
         * @return Transition table describing valid state transitions and actions.
         */
        auto operator()()
        {
            using namespace boost::sml;

            return make_transition_table(
                *state<Idle> + event<TaskAssigned> = state<Busy>,            ///< Idle → Busy on task assignment
                state<Busy> + event<TaskAssigned> = state<Busy>,///<get over ride mission
                state<Busy> + event<TaskCompleted> = state<Idle>, ///< Busy → MissionComplete on task completion
                state<Busy> + event<TaskAborted> = state<MissionAbort>,      ///< Busy → MissionAbort on task aborted
                state<MissionAbort> + event<TaskCompleted> = state<Idle>     ///< MissionAbort → Idle for next assignment
            );
        }
    };

    /**
     * @class CommandStateMachine
     * @brief Encapsulates the command state machine logic and handles task management.
     */
    class CommandStateMachine
    {
    public:
        using StateChangeSignal = boost::signals2::signal<void(drone_sdk::CommandStatus)>;
        using PathWayPoint = boost::signals2::signal<void(drone_sdk::Location)>;

        using CurrenitDestinationSignal = boost::signals2::signal<void(drone_sdk::Location)>;
        using LandingSignal = boost::signals2::signal<void(bool)>;
        using TakingOffSignal = boost::signals2::signal<void(bool)>;
        /**
         * @brief Constructs the CommandStateMachine instance.
         */
        CommandStateMachine();
        ~CommandStateMachine() = default;

        /**
         * @brief Handles task assignment events.
         * @param newMission The new mission type to assign.
         * @param singleDestination An optional single destination for the mission.
         * @param pathDestinations An optional queue of destinations for a path mission.
         * @throws std::invalid_argument If parameters are invalid for the mission type.
         */
        drone_sdk::FlightControllerStatus  handleTaskAssigned(
            drone_sdk::CurrentMission newMission,
            const std::optional<drone_sdk::Location> &singleDestination = std::nullopt,
            const std::optional<std::queue<drone_sdk::Location>> &pathDestinations = std::nullopt);

        /**
         * @brief Handles updates to the GPS location.
         * @param newLocation The updated GPS location of the drone.
         */
        void handleGpsLocationUpdate(const drone_sdk::Location &newLocation);
        void handleGpsStateChange(drone_sdk::safetyState gpsState);
        void handleLinkStateChange(drone_sdk::safetyState linkState);

        /**
         * @brief Sets the home base location for the drone.
         * @param newHome The new home base location.
         */
        void setHomebase(const drone_sdk::Location &newHome)
        {
            m_home = newHome;
        }

        /**
         * @brief Retrieves the current home base location.
         * @return The current home base location.
         */
        drone_sdk::Location getHomebase() const
        {
            return m_home;
        };

        /**
         * @brief Subscribe to command state changes.
         * @param subscriber A callback function to be triggered on state change.
         * @return A connection object for managing the subscription.
         */
        boost::signals2::connection subscribeToState(const StateChangeSignal::slot_type &subscriber);

        /**
         * @brief Subscribe to getting destination updates on path/ just an inner system funciton!
         * @param subscriber A callback function to be triggered on state change.
         * @return A connection object for managing the subscription.
         */
        boost::signals2::connection subscribeToCurrentDestination(const CurrenitDestinationSignal::slot_type &subscriber);
        /**
         * @brief Subscribe to path waypoint updates.
         * @param subscriber A callback function to be triggered when a new waypoint is reached.
         * @return A connection object for managing the subscription.
         */
        boost::signals2::connection subscribeToPathWaypoint(const PathWayPoint::slot_type &subscriber);

        boost::signals2::connection subscribeToLandingSignal(const LandingSignal::slot_type &subscriber);

        boost::signals2::connection subscribeToTakingOffSignal(const TakingOffSignal::slot_type &subscriber);

        /**
         * @brief Retrieves the current command state.
         * @return The current state of the command state machine.
         */
        drone_sdk::CommandStatus getCurrentState() const
        {
            return m_currentState;
        }

    private:
        /**
         * @brief Handles task aborted events due to safety conditions.
         */
        void handleTaskAbortedSafety();

        /**
         * @brief Handles task completion events.
         */
        void handleTaskCompleted();

        /**
         * @brief Handles updates to the path during a PATH mission.
         */
        void handleTaskPathUpdate();

        /**
         * @brief Updates the current state of the command state machine.
         */
        void updateCurrentState();

        /**
         * @brief Updates the current mission type.
         * @param newMission The new mission type to update.
         */
        void updateCurrentDestination(drone_sdk::Location newDestination);
        void takingOffCheck()
        {
            if (m_currLocation.altitude == m_home.altitude)
            {
                m_takingOffSignal(true);
            }
        }

    private:
        boost::sml::sm<Command_SM> m_SM;         ///< Boost.SML state machine instance.
        drone_sdk::CommandStatus m_currentState; ///< Current state of the state machine.

        StateChangeSignal m_stateChangeSignal; ///< Signal for state change notifications.
        PathWayPoint m_pathWaypoint;           ///< Signal for waypoint updates.
        CurrenitDestinationSignal m_currDestinationSignal;
        LandingSignal m_landingSignal;
        TakingOffSignal m_takingOffSignal;

        drone_sdk::CurrentMission m_currMission;     ///< Current mission type.
        drone_sdk::Location m_currLocation;          ///< Current GPS location of the drone.
        drone_sdk::Location m_destination;           ///< Current destination for the mission.
        drone_sdk::Location m_home;                  ///< Home base location.
        std::queue<drone_sdk::Location> m_pathQueue; ///< Queue of waypoints for a PATH mission.
    };

} // namespace commandstatemachine

#endif // COMMAND_STATE_MACHINE_HPP
