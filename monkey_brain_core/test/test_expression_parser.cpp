#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/parse_expression.hpp"

#include "monkey_brain_core/expression_description_helper.hpp"

namespace mbc = monkey_brain_core;

using ::testing::Eq;
using ::testing::Optional;

struct ParseExpressionTestCase
{
  std::string str;
  mbc::ExpressionDescription expected;
  std::string name;
};

using ParseExpression = ::testing::TestWithParam<ParseExpressionTestCase>;
using PETC = ParseExpressionTestCase;

TEST(parse_expression, parses_empty_string_as_nullopt) {
  const auto cond = monkey_brain_core::parse_expression("");
  ASSERT_FALSE(cond);
}

TEST_P(ParseExpression, parses) {
  const auto cond = mbc::parse_expression(GetParam().str);
  ASSERT_THAT(cond, Optional(Eq(GetParam().expected)));
}

namespace
{

std::string fix_test_name(const testing::TestParamInfo<ParseExpression::ParamType> & info)
{
  return info.param.name;
}

} // namespace

/* *INDENT-OFF* */
INSTANTIATE_TEST_SUITE_P(
  AllOf, ParseExpression,
  ::testing::Values(
    PETC{"/out_topic/data",                   REF("/out_topic/data"),                                                      "references_as_VALUE"},
    PETC{"NOT a/ref",                         NOT(REF("a/ref")),                                                           "NOT_expression"},
    PETC{"(((NOT a/ref)))",                   NOT(REF("a/ref")),                                                           "NOT_expression_multiple_braces"},
    PETC{"a/ref AND b/ref",                   AND(REF("a/ref"), REF("b/ref")),                                             "AND_expression"},
    PETC{"a/ref % b/ref",                     MODULO(REF("a/ref"), REF("b/ref")),                                          "modulo_expression"},
    PETC{"(a/ref % b/ref)",                   MODULO(REF("a/ref"), REF("b/ref")),                                          "modulo_expression_braced"},
    PETC{"a/ref == b/ref",                    EQUALS(REF("a/ref"), REF("b/ref")),                                          "equals_expression"},
    PETC{"(a/ref % 5) == 0",                  EQUALS(MODULO(REF("a/ref"), VALUE(5L)), VALUE(0L)),                          "complex_expression_with_modulo_braced"},
    PETC{"a/ref % 5 == 0",                    EQUALS(MODULO(REF("a/ref"), VALUE(5L)), VALUE(0L)),                          "complex_expression_with_modulo"},
    PETC{"a/ref == b/ref AND c/ref == d/ref", AND(EQUALS(REF("a/ref"), REF("b/ref")), EQUALS(REF("c/ref"), REF("d/ref"))), "complex_expression_with_comparisons"},
    PETC{"a/ref + b/ref + c/ref == 0",        EQUALS(PLUS(REF("a/ref"), REF("b/ref"), REF("c/ref")), VALUE(0L)),           "multiple_additions_as_one"},
    PETC{"a/ref +  50",                       PLUS(REF("a/ref"), VALUE(50L)),                                              "non_boolean_expression"},
    PETC{"(a/ref +  50)",                     PLUS(REF("a/ref"), VALUE(50L)),                                              "non_boolean_expression_braces"},
    PETC{"fun(a/ref)",                        FUN(REF("a/ref")),                                                           "unary_function"},
    PETC{"fun(a/ref, 2, 3)",                  FUN(REF("a/ref"), VALUE(2L), VALUE(3L)),                                     "nary_function"},
    PETC{"fun(gun(1), gun(fun(2, 3, 4)), 5)", FUN(GUN(VALUE(1L)), GUN(FUN(VALUE(2L), VALUE(3L), VALUE(4L))), VALUE(5L)),   "nested_functions"},
    PETC{"fun(1) == 0",                       EQUALS(FUN(VALUE(1L)), VALUE(0L)),                                           "functions_as_predicates"},
    PETC{"fun(1 == a)",                       FUN(EQUALS(VALUE(1L), REF("a"))),                                            "functions_that_take_boolean_conditions"},
    PETC{"UNSIGNED(1)",                       UNSIGNED(VALUE(1L)),                                                         "cast_to_UNSIGNED"},
    PETC{"UNSIGNED(1) == 1u",                 EQUALS(UNSIGNED(VALUE(1L)), VALUE(1UL)),                                     "cast_to_UNSIGNED_in_condition"},
    PETC{"1U",                                VALUE(1UL),                                                                  "unsigned_integer"},
    PETC{"3.14",                              VALUE(3.14),                                                                 "double"},
    PETC{"\"a string\"",                      STR("a string"),                                                             "quoted_strings_as_strings"}),
  fix_test_name);
/* *INDENT-ON* */

TEST(parse_expression, throws_when_expression_is_invalid) {
  ASSERT_THROW(monkey_brain_core::parse_expression("AND"), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("NOT"), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("OR"), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("||"), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("a/ref AND"), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("AND a/ref"), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("a/ref =="), std::invalid_argument);
  ASSERT_THROW(monkey_brain_core::parse_expression("== a/ref"), std::invalid_argument);
}
