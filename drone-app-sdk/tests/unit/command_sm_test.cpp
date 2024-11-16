#include "state_machines/command_state_machine.hpp"
#include "icd.hpp"

#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>
#include <queue>
#include <optional>

using namespace boost::placeholders;
using namespace commandstatemachine;

// Helper class to observe state changes
class CommandStateChangeObserver {
public:
    void onStateChanged(drone_sdk::CommandStatus newState) {
        lastState = newState;
    }

    drone_sdk::CommandStatus getLastState() const { return lastState; }

private:
    drone_sdk::CommandStatus lastState = drone_sdk::CommandStatus::IDLE;
};

// Test fixture
class CommandStateMachineTest : public ::testing::Test {
protected:
    CommandStateMachine csm;
    CommandStateChangeObserver observer;

    void SetUp() override {
        // Subscribe the observer to state change signals
        csm.subscribeToState(
            boost::bind(&CommandStateChangeObserver::onStateChanged, &observer, _1));
    }
};

// Test: Initial state
TEST_F(CommandStateMachineTest, InitialState) {
    EXPECT_EQ(csm.getCurrentState(), drone_sdk::CommandStatus::IDLE);
}
