#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/build_condition.hpp"
#include "monkey_brain_core/build_expression.hpp"
#include "monkey_brain_core/io_plugin_registry.hpp"

#include "monkey_brain_core/expression_description_helper.hpp"
#include "monkey_brain_core/input_fake.hpp"

namespace mbc = monkey_brain_core;

const std::string kBoolReference{"reference/to/bool"};
const std::string kInt32Reference{"reference/to/int32"};
const std::int32_t kInt32Value = 42;

struct ExpressionTestCase
{
  mbc::ExpressionDescription const description;
  std::int32_t expected_result;
};

class BuildExpression : public ::testing::TestWithParam<ExpressionTestCase>
{
public:
  BuildExpression()
  {
    env.add_plugin(
      std::make_unique<mbc::InputFake<std::int32_t>>(
        kInt32Value, mbc::ValueTypes::INT32, kInt32Reference));
  }

protected:
  mbc::IOPluginRegistry env;
};

TEST_P(BuildExpression, builds_working_expression) {
  auto const & description = GetParam().description;
  auto expr = mbc::build_expression(description, env);
  ASSERT_EQ(expr.type, mbc::ValueTypes::INT64);
  ASSERT_NE(nullptr, expr.expression);
  auto value_expression = dynamic_cast<mbc::Expression<std::int64_t> *>(expr.expression.get());
  EXPECT_EQ(GetParam().expected_result, value_expression->get());
}

const mbc::ExpressionDescription kInt32Expression{REF(kInt32Reference)};
const mbc::ExpressionDescription kConstantExpression{VALUE(42L)};
const mbc::ExpressionDescription kZeroExpression{VALUE(0L)};
const mbc::ExpressionDescription kModuloExpression{MODULO(kInt32Expression, kConstantExpression)};

const mbc::ExpressionDescription kTrippleSum{PLUS(kInt32Expression, kInt32Expression,
    kInt32Expression)};
const mbc::ExpressionDescription kThreeTimes42{VALUE(3L * 42L)};

INSTANTIATE_TEST_SUITE_P(
  AllOf, BuildExpression,
  ::testing::Values(
    ExpressionTestCase{kZeroExpression, 0},
    ExpressionTestCase{kInt32Expression, kInt32Value},
    ExpressionTestCase{kTrippleSum, kInt32Value * 3},
    ExpressionTestCase{kModuloExpression, 0}));

struct ConditionTestCase
{
  mbc::ExpressionDescription const description;
  bool expected_result;
};

class BuildCondition : public ::testing::TestWithParam<ConditionTestCase>
{
public:
  BuildCondition()
  {
    env.add_plugin(
      std::make_unique<mbc::InputFake<bool>>(
        true, mbc::ValueTypes::BOOL, kBoolReference));
    env.add_plugin(
      std::make_unique<mbc::InputFake<std::int32_t>>(
        kInt32Value, mbc::ValueTypes::INT32, kInt32Reference));
  }

protected:
  mbc::IOPluginRegistry env;
};

TEST_P(BuildCondition, builds_working_condition) {
  mbc::ExpressionDescription const & description = GetParam().description;
  auto condition = mbc::build_condition(description, env);
  ASSERT_NE(nullptr, condition);
  EXPECT_EQ(GetParam().expected_result, condition->holds());
}

const mbc::ExpressionDescription kBooleanConditionConstant{VALUE(false)};
const mbc::ExpressionDescription kBooleanCondition{REF(kBoolReference)};
const mbc::ExpressionDescription kNotCondition{NOT(kBooleanCondition)};
const mbc::ExpressionDescription kAndCondition{AND(kBooleanCondition, kBooleanCondition)};
const mbc::ExpressionDescription kFalseAndCondition{AND(kNotCondition, kBooleanCondition)};
const mbc::ExpressionDescription kOrCondition{OR(kNotCondition, kBooleanCondition)};

const mbc::ExpressionDescription kIdentityComparisonCondition{EQUALS(kInt32Expression,
    kInt32Expression)};
const mbc::ExpressionDescription kComparisonToConstantCondition{EQUALS(kInt32Expression,
    kConstantExpression)};
const mbc::ExpressionDescription kComparisonToConstantUnequalCondition{NEQ(kInt32Expression,
    kConstantExpression)};
const mbc::ExpressionDescription kModuloEqualsZeroCondition{EQUALS(kModuloExpression,
    kZeroExpression)};

const mbc::ExpressionDescription kTrippleSumEqualsThreeTime42{EQUALS(kTrippleSum, kThreeTimes42)};

INSTANTIATE_TEST_SUITE_P(
  AllOf, BuildCondition,
  ::testing::Values(
    ConditionTestCase{kBooleanConditionConstant, false},
    ConditionTestCase{kBooleanCondition, true},
    ConditionTestCase{kNotCondition, false},
    ConditionTestCase{kAndCondition, true},
    ConditionTestCase{kFalseAndCondition, false},
    ConditionTestCase{kOrCondition, true},
    ConditionTestCase{kIdentityComparisonCondition, true},
    ConditionTestCase{kComparisonToConstantCondition, true},
    ConditionTestCase{kComparisonToConstantUnequalCondition, false},
    ConditionTestCase{kTrippleSumEqualsThreeTime42, true},
    ConditionTestCase{kModuloEqualsZeroCondition, true},
    ConditionTestCase{EQUALS(UNSIGNED(VALUE(1L)), VALUE(uint64_t{1UL})), true}
));
