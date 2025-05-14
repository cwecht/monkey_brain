#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_scxml/build_states.hpp"

#include "monkey_brain_core/environment_mock.hpp"
#include "monkey_brain_core/operations_factory_mock.hpp"

using testing::ElementsAre;
using testing::NiceMock;

using namespace monkey_brain_scxml;

class BuildStates : public ::testing::Test
{
protected:
  NiceMock<monkey_brain_core::OperationsFactoryMock> ops_factory;
};

using StateDescriptions = std::vector<StateDescription>;

TEST_F(BuildStates, throws_on_empty_descriptions) {
  ASSERT_THROW(build_states({}, ops_factory), std::runtime_error);
}

TEST_F(BuildStates, connects_target_states) {
  StateDescriptions state_descriptions;
  state_descriptions.push_back(StateDescription{"a", {{"event", {}, {}, "a"}}, {}, {}, {}});

  auto states = build_states({std::move(state_descriptions), "a", {}}, ops_factory);
  ASSERT_EQ(states.states.size(), 1UL);
  auto const & s = states.states.front();
  ASSERT_THAT(states.initial_states, ElementsAre(&s));
  ASSERT_THAT(s.transitions.front().target_state, ElementsAre(&s));
}

TEST_F(BuildStates, first_state_as_initial_state) {
  StateDescriptions state_descriptions;
  state_descriptions.push_back(StateDescription{"a", {{"event", {}, {}, "a"}}, {}, {}, {}});

  auto states = build_states({std::move(state_descriptions), "", {}}, ops_factory);
  ASSERT_EQ(states.states.size(), 1UL);
  auto const & s = states.states.front();
  ASSERT_THAT(states.initial_states, ElementsAre(&s));
}
