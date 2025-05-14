#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_scxml/build_states.hpp"
#include "monkey_brain_scxml/parse_scxml.hpp"
#include "monkey_brain_scxml/state_machine.hpp"

#include "monkey_brain_core/environment_mock.hpp"
#include "monkey_brain_core/operations_factory_mock.hpp"

using namespace monkey_brain_scxml;

namespace
{

template<typename T>
T getenv(const char * var)
{
  const char * envvar = ::getenv(var);
  if (envvar == nullptr) {
    throw std::runtime_error("environment variable '" + std::string{var} + "' not set!");
  }
  return T{envvar};
}

const auto kTestDataFolder{getenv<std::filesystem::path>("TEST_DATA_FOLDER")};

std::unique_ptr<StateMachine> make_state_machine(const std::filesystem::path & scxml_path)
{
  static monkey_brain_core::EnvironmentMock env;
  static monkey_brain_core::OperationsFactoryMock ops_factory;
  auto states = build_states(parse_scxml(scxml_path), ops_factory);
  auto sm = std::make_unique<StateMachine>();
  sm->initialize(std::move(states));
  return sm;
}
}  // namespace

using testing::UnorderedElementsAre;

TEST(WikiStateMachine, is_initialy_in_ready) {
  auto sm = make_state_machine(kTestDataFolder / "wiki.scxml");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("ready"));
}

TEST(WikiStateMachineInReady, transitions_to_running_after_watch_start) {
  auto sm = make_state_machine(kTestDataFolder / "wiki.scxml");
  sm->post_event("watch.start");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("running"));
}

TEST(WikiStateMachineInRunning, transitions_to_paused_after_watch_split) {
  auto sm = make_state_machine(kTestDataFolder / "wiki.scxml");
  sm->post_event("watch.start");
  sm->post_event("watch.split");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("paused"));
}

TEST(FinalStateExampleStateMachine, is_initialy_in_S11_and_S21) {
  auto sm = make_state_machine(kTestDataFolder / "final_state_example.scxml");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("p", "S1", "S11", "S2", "S21"));
}

TEST(FinalStateExampleInS11S21StateMachine, transitions_to_S1Final_and_S22_after_e4e1) {
  auto sm = make_state_machine(kTestDataFolder / "final_state_example.scxml");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("p", "S1", "S11", "S2", "S21"));
  sm->post_event("e4");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("p", "S1", "S12", "S2", "S21"));
  sm->post_event("e1");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("p", "S1", "S1Final", "S2", "S22"));
}

TEST(FinalStateExampleInS11S21StateMachine, transitions_to_someOtherState_after_e4e1e2) {
  auto sm = make_state_machine(kTestDataFolder / "final_state_example.scxml");
  sm->post_event("e4");
  sm->post_event("e1");
  sm->post_event("e2");
  ASSERT_THAT(sm->active_states(), UnorderedElementsAre("someOtherState"));
}

TEST(StateMachineWithMultipleInitialStates, is_initialized_with_correcto_states) {
  auto sm = make_state_machine(kTestDataFolder / "initial_multiple_states.scxml");
  ASSERT_THAT(
    sm->active_states(),
    UnorderedElementsAre("second", "parallel", "first_parallel", "b", "second_parallel", "d"));
}
