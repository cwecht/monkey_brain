#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/build_expression.hpp"
#include "monkey_brain_core/build_condition.hpp"
#include "monkey_brain_core/io_plugin_registry.hpp"
#include "monkey_brain_core/parse_expression.hpp"

#include "monkey_brain_core/input_fake.hpp"

namespace mbc = monkey_brain_core;

const std::string kBoolReference{"reference/to/bool"};
const std::string kInt32Reference{"reference/to/int32"};
const std::int32_t kInt32Value = 42;

class BuildExpressionBase
{
public:
  BuildExpressionBase()
  {
    env.add_plugin(
      std::make_unique<mbc::InputFake<std::int32_t>>(
        kInt32Value, mbc::ValueTypes::INT32, kInt32Reference));
  }

protected:
  mbc::IOPluginRegistry env;
};

struct ExpressionTestCase
{
  std::string const str;
  std::int32_t expected_result;
};

struct BuildExpression : ::testing::TestWithParam<ExpressionTestCase>,
    BuildExpressionBase
{
  std::unique_ptr<mbc::Expression<std::int64_t>> create_expression(const std::string & str)
  {
    auto expr = mbc::build_expression(mbc::parse_expression(str).value(), env);
    EXPECT_EQ(mbc::ValueTypes::INT64, expr.type);
    return std::unique_ptr<mbc::Expression<std::int64_t>>(
      dynamic_cast<mbc::Expression<std::int64_t> *>(
        expr.expression.release()));
  }
};

TEST_P(BuildExpression, builds_working_expression) {
  auto expression = create_expression(GetParam().str);
  ASSERT_NE(nullptr, expression);
  EXPECT_EQ(GetParam().expected_result, expression->get());
}

INSTANTIATE_TEST_SUITE_P(
  AllOf, BuildExpression,
  ::testing::Values(
    ExpressionTestCase{"reference/to/int32", kInt32Value},
    ExpressionTestCase{"(reference/to/int32 + 5)", kInt32Value + 5},
    ExpressionTestCase{"2 * (reference/to/int32 + 50)", (kInt32Value + 50) * 2},
    ExpressionTestCase{"reference/to/int32 % 50", kInt32Value % 50},
    ExpressionTestCase{"50 / reference/to/int32", 50 / kInt32Value},
    ExpressionTestCase{"reference/to/int32 / 2 / 3", 7},
    ExpressionTestCase{"2 * reference/to/int32 * 2", 2 * 42 * 2}));

struct ConditionTestCase
{
  std::string const str;
  bool expected_result;
};

struct BuildCondition : ::testing::TestWithParam<ConditionTestCase>,
    BuildExpressionBase
{
  mbc::ConditionPtr create_condition(const std::string & str)
  {
    return mbc::build_condition(mbc::parse_expression(str).value(), env);
  }
};

TEST_P(BuildCondition, builds_working_condition) {
  auto condition = create_condition(GetParam().str);
  ASSERT_NE(nullptr, condition);
  EXPECT_EQ(GetParam().expected_result, condition->holds());
}

INSTANTIATE_TEST_SUITE_P(
  AllOf, BuildCondition,
  ::testing::Values(
    ConditionTestCase{"reference/to/int32 == 42", true},
    ConditionTestCase{"reference/to/int32 != 41", true},
    ConditionTestCase{"!(reference/to/int32 == 41)", true},
    ConditionTestCase{"(reference/to/int32 + 6 + 2) == 50", true},
    ConditionTestCase{"reference/to/int32 + 6 + 2 == 50", true},
    ConditionTestCase{"6 + reference/to/int32 + 2 == 50", true},
    ConditionTestCase{"reference/to/int32 == SIGNED(42u)", true},
    ConditionTestCase{"UNSIGNED(reference/to/int32) == 42u", true},
    ConditionTestCase{"UNSIGNED(42.0) == 42u", true},
    ConditionTestCase{"reference/to/int32 == 41", false}));
