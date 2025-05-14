#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/cli_context.hpp"

TEST(CLIContext, is_always_a_dry_run) {
  monkey_brain_core::CLIContext cc{0, nullptr};
  ASSERT_TRUE(cc.is_dry_run());
}

TEST(CLIContext, parses_string_parameter) {
  const std::array<const char *, 1> params = {"parameter_name:=value"};
  monkey_brain_core::CLIContext cc{params.size(), params.data()};
  ASSERT_EQ(cc.declare_parameter<std::string>("parameter_name"), "value");
}

template<typename T>
using ClIContextParses = ::testing::TestWithParam<std::pair<std::string, T>>;

using CLIContextParses_integer = ClIContextParses<int64_t>;

TEST_P(CLIContextParses_integer, parameter) {
  auto [str, value] = GetParam();
  const std::string arg = "parameter_name:=" + str;
  const std::array<const char *, 1> params = {arg.c_str()};
  monkey_brain_core::CLIContext cc{params.size(), params.data()};
  ASSERT_EQ(cc.declare_parameter<int64_t>("parameter_name"), value);
}

INSTANTIATE_TEST_SUITE_P(
  AllOf, CLIContextParses_integer,
  ::testing::Values(
    std::make_pair("5", 5),
    std::make_pair("05", 5),
    std::make_pair("0", 0),
    std::make_pair("-5", -5),
    std::make_pair("-05", -5),
    std::make_pair("9223372036854775807", std::numeric_limits<int64_t>::max()),
    std::make_pair("-9223372036854775808", std::numeric_limits<int64_t>::min())
));

using CLIContextParses_double = ClIContextParses<double>;

TEST_P(CLIContextParses_double, parameter) {
  auto [str, value] = GetParam();
  const std::string arg = "parameter_name:=" + str;
  const std::array<const char *, 1> params = {arg.c_str()};
  monkey_brain_core::CLIContext cc{params.size(), params.data()};
  ASSERT_EQ(cc.declare_parameter<double>("parameter_name"), value);
}

INSTANTIATE_TEST_SUITE_P(
  AllOf, CLIContextParses_double,
  ::testing::Values(
    std::make_pair("5", 5),
    std::make_pair("5.5", 5.5),
    std::make_pair("05", 5),
    std::make_pair("0", 0),
    std::make_pair("0.0", 0),
    std::make_pair("-5", -5),
    std::make_pair("-5.5", -5.5),
    std::make_pair("-05", -5),
    std::make_pair("9223372036854775807", std::numeric_limits<int64_t>::max()),
    std::make_pair("-9223372036854775808", std::numeric_limits<int64_t>::min()),
    std::make_pair("1.7976931348623157e+308", std::numeric_limits<double>::max()),
    std::make_pair("-1.7976931348623157e+308", std::numeric_limits<double>::lowest()),
    std::make_pair("2.2250738585072014e-308", std::numeric_limits<double>::min())
));

using CLIContextParses_boolean = ClIContextParses<bool>;

TEST_P(CLIContextParses_boolean, parameter) {
  auto [str, value] = GetParam();
  const std::string arg = "parameter_name:=" + str;
  const std::array<const char *, 1> params = {arg.c_str()};
  monkey_brain_core::CLIContext cc{params.size(), params.data()};
  ASSERT_EQ(cc.declare_parameter<bool>("parameter_name"), value);
}

INSTANTIATE_TEST_SUITE_P(
  AllOf, CLIContextParses_boolean,
  ::testing::Values(
    std::make_pair("true", true),
    std::make_pair("false", false)
));
