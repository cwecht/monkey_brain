#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_scxml/validate_state_machine.hpp"

#include "monkey_brain_core/environment_mock.hpp"

using namespace monkey_brain_scxml;
using ::testing::Return;
using ::testing::IsEmpty;
using ::testing::SizeIs;

TEST(validate_state_machine, finds_no_errors_empty_environment_and_empty_statemachine) {
  StateMachineDescription sds;
  monkey_brain_core::EnvironmentMock env;
  EXPECT_CALL(env, get_all_events).Times(1);

  ASSERT_THAT(validate_state_machine(sds, env), IsEmpty());
}

TEST(validate_state_machine, finds_no_errors_for_defined_but_not_referenced_events) {
  StateMachineDescription sds;
  sds.states.push_back(StateDescription{"init", {}, {}, {}, {}});
  sds.initial.state = "init";
  sds.initial.action.push_back({"RAISE", {{"VALUE", {}, {std::string{"raised_event"}}}}});
  monkey_brain_core::EnvironmentMock env;
  EXPECT_CALL(env, get_all_events).WillOnce(Return(std::unordered_set<std::string>{"env_event"}));

  ASSERT_THAT(validate_state_machine(sds, env), IsEmpty());
}

TEST(validate_state_machine, finds_no_errors_for_defined_and_referenced_events) {
  StateMachineDescription sds;
  sds.states.push_back(StateDescription{"init",
      {{"raised_event", {}, {}, ""}, {"env_event", {}, {}, ""}}, {}, {}, {}});
  sds.initial.state = "init";
  sds.initial.action.push_back({"RAISE", {{"VALUE", {}, {std::string{"raised_event"}}}}});
  monkey_brain_core::EnvironmentMock env;
  EXPECT_CALL(env, get_all_events).WillOnce(Return(std::unordered_set<std::string>{"env_event"}));

  ASSERT_THAT(validate_state_machine(sds, env), IsEmpty());
}

TEST(validate_state_machine, finds_errors_for_undefined_but_referenced_events) {
  StateMachineDescription sds;
  sds.states.push_back(StateDescription{"init",
      {{"not_raised_event", {}, {}, ""}, {"not_env_event", {}, {}, ""}}, {}, {}, {}});
  sds.initial.state = "init";
  sds.initial.action.push_back({"RAISE", {{"VALUE", {}, {std::string{"raised_event"}}}}});
  monkey_brain_core::EnvironmentMock env;
  EXPECT_CALL(env, get_all_events).WillOnce(Return(std::unordered_set<std::string>{"env_event"}));

  ASSERT_THAT(validate_state_machine(sds, env), SizeIs(2));
}
