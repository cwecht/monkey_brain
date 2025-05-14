#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_scxml/state_machine.hpp"
#include "monkey_brain_scxml/state_builder.hpp"

#include "monkey_brain_core/environment_mock.hpp"

using namespace monkey_brain_scxml;
using testing::ElementsAre;
using testing::InSequence;

struct StateMachineObserverMock : StateMachineObserver
{
  MOCK_METHOD1(initialize, void(const std::vector<State> &));
  MOCK_METHOD1(on_state_change, void(StateNameRange));
};

std::unique_ptr<StateMachineObserverMock> make_state_machine_observer_mock()
{
  auto obs = std::make_unique<StateMachineObserverMock>();
  EXPECT_CALL(*obs, initialize).Times(1);
  return obs;
}

struct AnInitializedStateMachine : public ::testing::Test
{
  AnInitializedStateMachine()
  {
    EXPECT_CALL(env, add_internal_event_recipient).Times(1);
    EXPECT_CALL(env, set_event_recipient).Times(1);
  }
  monkey_brain_core::EnvironmentMock env;
};

TEST_F(AnInitializedStateMachine, is_in_its_first_state_initially) {
  States states;
  const auto & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  states.initial_states = {&first_state};
  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  EXPECT_THAT(sm.active_states(), ElementsAre("first_state"));
}

TEST_F(AnInitializedStateMachine, calls_on_shut_down_when_toplevel_final_state_is_reached) {
  States states;
  const auto & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  states.initial_states = {&first_state};
  states.states.front().type = State::Type::FINAL;
  testing::MockFunction<void()> on_shutdown;
  EXPECT_CALL(on_shutdown, Call()).Times(1);
  StateMachine sm{on_shutdown.AsStdFunction()};
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
}

TEST_F(AnInitializedStateMachine, performs_on_entry_action_for_initial_state) {
  auto action = std::make_unique<ActionMock>();
  EXPECT_CALL(*action, execute).Times(1);
  States states;
  const auto & first_state =
    states.states.emplace_back(
    StateBuilder{"first_state"}.with_on_entry_action(
      std::move(
        action)).build());
  states.initial_states = {&first_state};
  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  // action mock holds
}

TEST_F(AnInitializedStateMachine, reacts_on_internal_event) {
  States states;
  states.states.reserve(2);
  auto & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  states.initial_states = {&first_state};
  auto & second_state = states.states.emplace_back(StateBuilder{"second_state"}.build());
  add_transition(first_state, "internal_event", {&second_state});
  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  sm.post_internal_event("internal_event");
  EXPECT_THAT(sm.active_states(), ElementsAre("first_state"));
  // usually post_internal_event is intended to be called from an action inside a post_event
  // call. therefore, post_internal_event does not perform a step on the statemachine on its
  // own.
  sm.post_event("");
  EXPECT_THAT(sm.active_states(), ElementsAre("second_state"));
  // action mock holds
}

TEST_F(AnInitializedStateMachine, enters_nested_initial_state) {
  States states;
  State & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  State & second_state = first_state.substates.emplace_back(StateBuilder{"second_state"}.build());
  second_state.parent_state = &first_state;
  first_state.automatic_sub_states = {&second_state};
  states.initial_states = {&first_state};
  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  EXPECT_THAT(sm.active_states(), ElementsAre("first_state", "second_state"));
  // obs mock holds
}

TEST_F(AnInitializedStateMachine, enters_grandchild_initial_state) {
  auto entry_first_state = std::make_unique<ActionMock>();
  auto entry_second_state = std::make_unique<ActionMock>();
  {
    InSequence s;
    EXPECT_CALL(*entry_first_state, execute).Times(1);
    EXPECT_CALL(*entry_second_state, execute).Times(1);
  }
  States states;
  State & first_state =
    states.states.emplace_back(
    StateBuilder{"first_state"}.with_on_entry_action(
      std::move(
        entry_first_state)).build());

  State & second_state =
    first_state.substates.emplace_back(
    StateBuilder{"second_state"}.with_on_entry_action(std::move(entry_second_state)).build());
  first_state.automatic_sub_states = {&second_state};
  second_state.parent_state = &first_state;
  states.initial_states = {&second_state};
  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  EXPECT_THAT(sm.active_states(), ElementsAre("first_state", "second_state"));
}

TEST_F(AnInitializedStateMachine, changes_state_when_the_respective_event_arrives) {
  States states;
  states.states.reserve(2);
  auto & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  const auto & second_state = states.states.emplace_back(StateBuilder{"second_state"}.build());
  add_transition(first_state, "event", {&second_state});
  states.initial_states = {&first_state};

  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  EXPECT_THAT(sm.active_states(), ElementsAre("first_state"));
  sm.post_event("event");
  EXPECT_THAT(sm.active_states(), ElementsAre("second_state"));
}

TEST_F(AnInitializedStateMachine, executes_action_when_the_respective_event_arrives) {
  States states;
  states.states.reserve(2);
  auto & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  auto & second_state = states.states.emplace_back(StateBuilder{"second_state"}.build());
  auto action = std::make_unique<ActionMock>();
  EXPECT_CALL(*action, execute).Times(1);
  add_transition(first_state, "event", {&second_state}, std::move(action));
  states.initial_states = {&first_state};

  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));

  sm.post_event("event");
  // action mock holds
}

TEST_F(AnInitializedStateMachine, performs_on_entry_and_exit_actions_for_parallel_substates) {
  States states;
  auto entry_a_sub = std::make_unique<ActionMock>();
  auto entry_b_sub = std::make_unique<ActionMock>();
  auto exit_a_sub = std::make_unique<ActionMock>();
  auto exit_b_sub = std::make_unique<ActionMock>();
  {
    InSequence s;
    EXPECT_CALL(*entry_a_sub, execute).Times(1);
    EXPECT_CALL(*entry_b_sub, execute).Times(1);
    EXPECT_CALL(*exit_b_sub, execute).Times(1);
    EXPECT_CALL(*exit_a_sub, execute).Times(1);
  }
  states.states.reserve(2);
  auto & final_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  auto & first_parallel = states.states.emplace_back(StateBuilder{"first_parallel"}.build());
  auto & sub_a = first_parallel.substates.emplace_back(
    StateBuilder{"a_sub"}.with_on_entry_action(std::move(entry_a_sub)).with_on_exit_action(
      std::move(
        exit_a_sub)).with_parent(&first_parallel).build());
  add_transition(sub_a, "event", {&final_state});
  first_parallel.substates.emplace_back(
    StateBuilder{"b_sub"}.with_on_entry_action(std::move(entry_b_sub)).with_on_exit_action(
      std::move(
        exit_b_sub)).with_parent(&first_parallel).build());
  states.initial_states = {&states.states.back()};

  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));

  sm.post_event("event");
  // action mocks hold
}


TEST_F(AnInitializedStateMachine, leaves_only_active_children) {
  States states;
  auto entry_a_sub = std::make_unique<ActionMock>();
  auto entry_b_sub = std::make_unique<ActionMock>();
  auto exit_a_sub = std::make_unique<ActionMock>();
  auto exit_b_sub = std::make_unique<ActionMock>();
  EXPECT_CALL(*entry_a_sub, execute).Times(1);
  EXPECT_CALL(*exit_a_sub, execute).Times(1);
  EXPECT_CALL(*entry_b_sub, execute).Times(0);
  EXPECT_CALL(*exit_b_sub, execute).Times(0);
  states.states.reserve(2);
  auto & final_state = states.states.emplace_back(StateBuilder{"final_state"}.build());
  auto & first_state = states.states.emplace_back(StateBuilder{"first_state"}.build());
  first_state.substates.reserve(2);
  auto & a_sub = first_state.substates.emplace_back(
    StateBuilder{"a_sub"}.with_on_entry_action(std::move(entry_a_sub)).with_on_exit_action(
      std::move(
        exit_a_sub)).with_parent(&first_state).build());
  add_transition(a_sub, "event", {&final_state});
  first_state.substates.emplace_back(
    StateBuilder{"b_sub"}.with_on_entry_action(std::move(entry_b_sub)).with_on_exit_action(
      std::move(
        exit_b_sub)).with_parent(&first_state).build());
  states.initial_states = {&states.states.back()};
  first_state.automatic_sub_states = {&a_sub};

  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));

  sm.post_event("event");
  // action mocks hold
}

TEST_F(
  AnInitializedStateMachine,
  performs_on_exit_actions_for_parallel_substates_over_multiple_levels) {
  States states;
  auto exit_a_sub = std::make_unique<ActionMock>();
  auto exit_b_sub = std::make_unique<ActionMock>();
  auto exit_c_sub = std::make_unique<ActionMock>();
  auto exit_d_sub = std::make_unique<ActionMock>();
  {
    InSequence s;
    EXPECT_CALL(*exit_d_sub, execute).Times(1);
    EXPECT_CALL(*exit_c_sub, execute).Times(1);
    EXPECT_CALL(*exit_b_sub, execute).Times(1);
    EXPECT_CALL(*exit_a_sub, execute).Times(1);
  }
  states.states.reserve(2);
  auto & final_state = states.states.emplace_back(StateBuilder{"final_state"}.build());
  auto & first_parallel = states.states.emplace_back(StateBuilder{"first_parallel"}.build());
  first_parallel.substates.reserve(2);
  auto & second_parallel = first_parallel.substates.emplace_back(
    StateBuilder{"a_sub"}.with_on_exit_action(std::move(exit_a_sub)).with_parent(
      &first_parallel).build());
  second_parallel.parent_state = &first_parallel;

  auto & b_sub = second_parallel.substates.emplace_back(
    StateBuilder{"b_sub"}.with_on_exit_action(std::move(exit_b_sub)).with_parent(
      &second_parallel).build());
  add_transition(b_sub, "event", {&final_state});
  b_sub.parent_state = &second_parallel;
  auto & c_sub = second_parallel.substates.emplace_back(
    StateBuilder{"c_sub"}.with_on_exit_action(std::move(exit_c_sub)).with_parent(
      &second_parallel).build());
  c_sub.parent_state = &second_parallel;

  auto & d_sub = first_parallel.substates.emplace_back(
    StateBuilder{"d_sub"}.with_on_exit_action(std::move(exit_d_sub)).with_parent(
      &first_parallel).build());
  d_sub.parent_state = &first_parallel;

  states.initial_states = {&first_parallel};

  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));

  sm.post_event("event");
  EXPECT_THAT(sm.active_states(), ElementsAre("final_state"));
  // action mocks hold
}

TEST_F(
  AnInitializedStateMachine,
  leaves_child_state_when_parent_state_is_left_and_event_did_not_apply_to_child) {
  States states;
  states.states.reserve(2);
  auto & first_parent = states.states.emplace_back(StateBuilder{"first_state"}.build());
  auto & second_state = states.states.emplace_back(StateBuilder{"second_state"}.build());
  State & child =
    first_parent.substates.emplace_back(StateBuilder{"child"}.with_parent(&first_parent).build());
  first_parent.automatic_sub_states = {&child};
  states.initial_states = {&first_parent};
  add_transition(first_parent, "event", {&second_state});
  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  sm.post_event("event");
  EXPECT_THAT(sm.active_states(), ElementsAre("second_state"));
  // obs mock holds
}

TEST_F(
  AnInitializedStateMachine,
  returns_to_previous_states_when_it_enters_history_state) {
  States states;
  states.states.reserve(2);
  auto & first_parent = states.states.emplace_back(StateBuilder{"parent"}.build());
  auto & second_state = states.states.emplace_back(StateBuilder{"second_state"}.build());
  first_parent.substates.reserve(3);
  State & first_child =
    first_parent.substates.emplace_back(
    StateBuilder{"first_child"}.with_parent(
      &first_parent).build());
  first_parent.automatic_sub_states = {&first_child};
  states.initial_states = {&first_parent};
  State & second_child = first_parent.substates.emplace_back(
    StateBuilder{"second_child"}.with_parent(
      &first_parent).build());
  add_transition(first_child, "to_second_child", {&second_child});
  add_transition(second_child, "to_second_state", {&second_state});
  State & history_child = first_parent.substates.emplace_back(
    StateBuilder{"history_child"}.with_parent(
      &first_parent).build());
  history_child.type = State::Type::HISTORY_SHALLOW;
  add_transition(second_state, "to_history", {&history_child});
  add_transition(history_child, "", {history_child.parent_state});

  StateMachine sm;
  env.add_internal_event_recipient(&sm);
  env.set_event_recipient(&sm);
  sm.initialize(std::move(states));
  EXPECT_THAT(sm.active_states(), ElementsAre("parent", "first_child"));
  sm.post_event("to_second_child");
  EXPECT_THAT(sm.active_states(), ElementsAre("parent", "second_child"));
  sm.post_event("to_second_state");
  EXPECT_THAT(sm.active_states(), ElementsAre("second_state"));
  sm.post_event("to_history");
  EXPECT_THAT(sm.active_states(), ElementsAre("parent", "second_child"));
}
