#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_scxml/parse_scxml.hpp"

#include "monkey_brain_core/expression_description_helper.hpp"

using namespace monkey_brain_scxml;
using testing::ElementsAre;
using monkey_brain_core::ActionDescription;
using monkey_brain_core::ExpressionDescription;

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
}  // namespace

TEST(parse_scxml, throws_when_invalid_path_is_passed) {
  ASSERT_THROW(parse_scxml("bla"), std::invalid_argument);
}

TEST(parse_scxml, parses_correct_number_of_states_from_wiki_example) {
  ASSERT_EQ(parse_scxml(kTestDataFolder / "wiki.scxml").states.size(), 4UL);
}

TEST(parse_scxml, parses_initial_state_from_wiki_example) {
  ASSERT_EQ(parse_scxml(kTestDataFolder / "wiki.scxml").initial.state, "ready");
}

TEST(parse_scxml, parses_onentry_and_oneexit_actions_from_onentry_onexit_example) {
  auto states = parse_scxml(kTestDataFolder / "onentry_onexit.scxml").states;
  ASSERT_EQ(states.size(), 1UL);
  auto & state = states.front();

  EXPECT_THAT(state.on_entry_action, ElementsAre(ActionDescription{"PERFORM", {REF("/onentry")}}));
  EXPECT_THAT(state.on_exit_action, ElementsAre(ActionDescription{"PERFORM", {REF("/onexit")}}));
}

TEST(parse_scxml,
  parses_multiple_onentry_and_oneexit_actions_from_multiple_onentry_onexit_in_document_order) {
  auto states = parse_scxml(kTestDataFolder / "onentry_onexit_multiple.scxml").states;
  ASSERT_EQ(states.size(), 1UL);
  auto & state = states.front();

  EXPECT_THAT(state.on_entry_action,
    ElementsAre(ActionDescription{"PERFORM", {REF("/onentry")}},
    ActionDescription{"PERFORM", {REF("/onentry2")}}));
  EXPECT_THAT(state.on_exit_action,
    ElementsAre(ActionDescription{"PERFORM", {REF("/onexit")}},
    ActionDescription{"PERFORM", {REF("/onexit2")}}));
}

TEST(parse_scxml, parses_onentry_and_onexit_with_raise_action) {
  auto states = parse_scxml(kTestDataFolder / "raise_elements.scxml").states;
  ASSERT_EQ(states.size(), 1UL);
  auto & state = states.front();

  EXPECT_THAT(state.on_entry_action,
    ElementsAre(ActionDescription{"RAISE", {REF("onentry_event")}}));
  EXPECT_THAT(state.on_exit_action, ElementsAre(ActionDescription{"RAISE", {REF("onexit_event")}}));
}

TEST(parse_scxml, parses_nested_states) {
  const auto sm = parse_scxml(kTestDataFolder / "nested_states.scxml");

  ASSERT_EQ(sm.initial.state, "outer");
  ASSERT_EQ(sm.states.size(), 1UL);
  ASSERT_TRUE(sm.states.front().nested);

  ASSERT_EQ(sm.states.front().nested->states.size(), 4UL);
  ASSERT_EQ(sm.states.front().nested->initial.state, "ready");
}

TEST(parse_scxml, parses_final_states) {
  const auto sm = parse_scxml(kTestDataFolder / "final.scxml");

  ASSERT_EQ(sm.states.size(), 1UL);
  const auto & state = sm.states.front();
  ASSERT_EQ(state.name, "running");
  ASSERT_TRUE(state.is_final());

  EXPECT_THAT(state.on_entry_action, ElementsAre(ActionDescription{":=", {REF("a"), REF("b")}}));
  EXPECT_THAT(state.on_exit_action, ElementsAre(ActionDescription{":=", {REF("c"), REF("d")}}));
}

TEST(parse_scxml, parses_history_states) {
  const auto sm = parse_scxml(kTestDataFolder / "history_simple.scxml");

  ASSERT_EQ(sm.states.size(), 2UL);

  ASSERT_EQ(sm.states.front().name, "hist");
  ASSERT_TRUE(sm.states.front().is_history());
  ASSERT_EQ(StateType::HISTORY_SHALLOW, sm.states.front().type);

  ASSERT_EQ(sm.states.back().name, "hist_deep");
  ASSERT_TRUE(sm.states.back().is_history());
  ASSERT_EQ(StateType::HISTORY_DEEP, sm.states.back().type);
  ASSERT_EQ(sm.states.back().transitions.size(), 1);
  ASSERT_EQ(sm.states.back().transitions.back().target, "target");
  ASSERT_EQ(sm.states.back().transitions.back().action.size(), 1);
}

TEST(parse_scxml, parses_initial_tag) {
  const auto sm = parse_scxml(kTestDataFolder / "initial_tag.scxml");

  ASSERT_EQ(sm.initial.state, "outer");
  ASSERT_EQ(sm.initial.action.size(), 1);
  EXPECT_THAT(sm.initial.action, ElementsAre(ActionDescription{"RAISE", {REF("init_sm")}}));
  ASSERT_EQ(sm.states.size(), 1UL);
}

TEST(parse_scxml, initial_state_is_optional) {
  const auto sm = parse_scxml(kTestDataFolder / "no_initial.scxml");

  ASSERT_EQ(sm.initial.state, "");
  ASSERT_TRUE(sm.initial.action.empty());
  ASSERT_EQ(sm.states.size(), 2UL);
}

TEST(parse_scxml, parses_multiple_initial_state) {
  const auto sm = parse_scxml(kTestDataFolder / "initial_multiple_states.scxml");

  ASSERT_EQ(sm.initial.state, "b d");
  ASSERT_EQ(sm.states.size(), 2UL);
}
