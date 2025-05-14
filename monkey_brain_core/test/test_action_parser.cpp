#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/parse_action.hpp"

#include "monkey_brain_core/expression_description_helper.hpp"

using ::testing::ElementsAre;

TEST(parse_action, throws_when_empty_string_is_passed) {
  ASSERT_TRUE(monkey_brain_core::parse_action("").empty());
}

TEST(parse_action, parses_a_single_assignment_action) {
  const auto actions = monkey_brain_core::parse_action("val1 := b");
  ASSERT_EQ(actions.size(), 1UL);
  ASSERT_EQ(actions.front().type, ":=");
  ASSERT_THAT(actions.front().arguments, ElementsAre(REF("val1"), REF("b")));
}

TEST(parse_action, parses_complex_assignment_action) {
  const auto actions = monkey_brain_core::parse_action("a := b + 2");
  ASSERT_EQ(actions.size(), 1UL);
  ASSERT_EQ(actions.front().type, ":=");
  ASSERT_THAT(actions.front().arguments, ElementsAre(REF("a"), PLUS(REF("b"), VALUE(2L))));
}

TEST(parse_action, parses_a_single_perform_action) {
  const auto actions = monkey_brain_core::parse_action("PERFORM b");
  ASSERT_EQ(actions.size(), 1UL);
  ASSERT_EQ(actions.front().type, "PERFORM");
  ASSERT_THAT(actions.front().arguments, ElementsAre(REF("b")));
}

TEST(parse_action, parses_a_single_raise_action) {
  const auto actions = monkey_brain_core::parse_action("RAISE b");
  ASSERT_EQ(actions.size(), 1UL);
  ASSERT_EQ(actions.front().type, "RAISE");
  ASSERT_THAT(actions.front().arguments, ElementsAre(REF("b")));
}

TEST(parse_action, parses_multiple_actions) {
  const auto actions = monkey_brain_core::parse_action("a := b; PERFORM a");
  ASSERT_EQ(actions.size(), 2UL);
  ASSERT_EQ(actions.front().type, ":=");
  ASSERT_EQ(actions.front().arguments.size(), 2);
  ASSERT_THAT(actions.front().arguments, ElementsAre(REF("a"), REF("b")));
  ASSERT_EQ(actions.back().type, "PERFORM");
  ASSERT_THAT(actions.back().arguments, ElementsAre(REF("a")));
}
