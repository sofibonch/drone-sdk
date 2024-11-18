#include "state_machines/command_state_machine.hpp"

namespace commandstatemachine
{

    using namespace boost::sml;

    // CommandStateMachine constructor
    CommandStateMachine::CommandStateMachine()
        : m_SM(), m_currentState(drone_sdk::CommandStatus::IDLE), m_currMission(drone_sdk::CurrentMission::HOVER) {}

    // Handle task assignment
    drone_sdk::FlightControllerStatus CommandStateMachine::handleTaskAssigned(
        drone_sdk::CurrentMission newMission,
        const std::optional<drone_sdk::Location> &singleDestination,
        const std::optional<std::queue<drone_sdk::Location>> &pathDestinations)
    {
        m_SM.process_event(TaskAssigned{});

        m_currMission = newMission;
        updateCurrentState();

        if (singleDestination && pathDestinations)
        {
            return drone_sdk::FlightControllerStatus::INVALID_COMMAND;
        }
        switch (newMission)
        {
        case drone_sdk::CurrentMission::GOTO:
            if (!singleDestination)
            {
                return drone_sdk::FlightControllerStatus::INVALID_COMMAND;
            }
            takingOffCheck();
            m_destination = *singleDestination;
            break;

        case drone_sdk::CurrentMission::HOME:
            std::cout << "GOING HOMW!!!!" << std::endl;
            m_destination = m_home;
            break;
        case drone_sdk::CurrentMission::HOVER:
            if (singleDestination || pathDestinations)
            {
                return drone_sdk::FlightControllerStatus::INVALID_COMMAND;
            }
            takingOffCheck();
            m_destination = m_currLocation;
            handleTaskCompleted();
            break;
        case drone_sdk::CurrentMission::PATH:
            if (!pathDestinations)
            {
                return drone_sdk::FlightControllerStatus::INVALID_COMMAND;
            }
            m_pathQueue = *pathDestinations;
            m_destination = m_pathQueue.front();
            m_pathQueue.pop();
            break;
        default:
            return drone_sdk::FlightControllerStatus::INVALID_COMMAND;
        }
        m_missionChangeSignal(newMission);
        return drone_sdk::FlightControllerStatus::SUCCESS;
    }

    // Handle task aborted due to safety
    void CommandStateMachine::handleTaskAbortedSafety()
    {
        m_SM.process_event(TaskAborted{});
        // go to the height of the home
        drone_sdk::Location emergancy_landing{m_currLocation.latitude, m_currLocation.longitude, m_home.altitude};
        m_destination = emergancy_landing;
        updateCurrentState();
    }

    // Subscribe to state changes
    boost::signals2::connection CommandStateMachine::subscribeToState(const StateChangeSignal::slot_type &subscriber)
    {
        return m_stateChangeSignal.connect(subscriber);
    }

    // Subscribe to path waypoint updates
    boost::signals2::connection CommandStateMachine::subscribeToPathWaypoint(const PathWayPoint::slot_type &subscriber)
    {
        return m_pathWaypoint.connect(subscriber);
    }

    boost::signals2::connection CommandStateMachine::subscribeToCurrentDestination(const CurrenitDestinationSignal::slot_type &subscriber)
    {
        return m_currDestinationSignal.connect(subscriber);
    }

    boost::signals2::connection CommandStateMachine::subscribeToLandingSignal(const LandingSignal::slot_type &subscriber)
    {
        return m_landingSignal.connect(subscriber);
    }

    boost::signals2::connection CommandStateMachine::subscribeToTakingOffSignal(const TakingOffSignal::slot_type &subscriber)
    {
        return m_takingOffSignal.connect(subscriber);
    }

    boost::signals2::connection CommandStateMachine::subscribeToMission(const MissionChangeSignal::slot_type &subscriber)
    {
        return m_missionChangeSignal.connect(subscriber);
    }

    // Handle GPS location updates
    void CommandStateMachine::handleGpsLocationUpdate(const drone_sdk::Location &newLocation)
    {
        m_currLocation = newLocation;
        if (m_currLocation == m_destination)
        {
            // std::cout<<"reached destination!!!!"<<m_currLocation.altitude<<std::endl;
            switch (m_currMission)
            {
            case drone_sdk::CurrentMission::GOTO:
                handleTaskCompleted();
                break;
            case drone_sdk::CurrentMission::HOME:
                LandingSignal();
                handleTaskCompleted();
                break;
            case drone_sdk::CurrentMission::PATH:
                handleTaskPathUpdate();
                break;
            case drone_sdk::CurrentMission::HOVER:
                // Remain in the current state
                break;
            case drone_sdk::CurrentMission::EMERGENCY:
                LandingSignal();
                handleTaskCompleted();
                break;
            default:
                break;
            }
        }
    }

    void CommandStateMachine::handleGpsStateChange(drone_sdk::safetyState gpsState)
    {
        if (gpsState == drone_sdk::safetyState::GPS_NOT_HEALTHY)
        {
            handleTaskAbortedSafety();
        }
    }

    void CommandStateMachine::handleLinkStateChange(drone_sdk::safetyState linkState)
    {
        if (linkState == drone_sdk::safetyState::NOT_CONNECTED)
        {
            handleTaskAbortedSafety();
        }
    }
    // Handle task completion
    void CommandStateMachine::handleTaskCompleted()
    {
        m_SM.process_event(TaskCompleted{});
        updateCurrentState();
    }

    // Handle path updates for PATH mission
    void CommandStateMachine::handleTaskPathUpdate()
    {
        m_pathWaypoint(m_destination);
        if (m_pathQueue.empty())
        {
            handleTaskCompleted();
        }
        else
        {
            updateCurrentDestination(m_pathQueue.front());
            m_pathQueue.pop();
        }
    }

    // Update current state
    void CommandStateMachine::updateCurrentState()
    {
        //       drone_sdk::CommandStatus prevState = m_currentState;

        if (m_SM.is(state<Idle>))
        {
            m_currentState = drone_sdk::CommandStatus::IDLE;
        }
        else if (m_SM.is(state<Busy>))
        {
            m_currentState = drone_sdk::CommandStatus::BUSY;
        }
        else if (m_SM.is(state<MissionAbort>))
        {
            m_currentState = drone_sdk::CommandStatus::MISSION_ABORT;
        }

        m_stateChangeSignal(m_currentState);
    }

    // Update current mission
    void CommandStateMachine::updateCurrentDestination(drone_sdk::Location newDestination)
    {
        m_destination = newDestination;
        m_currDestinationSignal(m_destination);
    }

} // namespace commandstatemachine
