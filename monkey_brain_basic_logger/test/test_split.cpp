#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_basic_logger/split.hpp"

using namespace testing;

struct SplitParamsAndResult
{
  std::string_view input;
  std::string_view delimiter;
  std::vector<std::string_view> expected_result;
};

using Split = ::testing::TestWithParam<SplitParamsAndResult>;

TEST_P(Split, splits_string_into_strings) {
  const auto [input, delimiter, expected_result] = GetParam();
  const auto r = monkey_brain_basic_logger::split(input, delimiter);
  ASSERT_THAT(r, ContainerEq(expected_result));
}

INSTANTIATE_TEST_SUITE_P(
  AllOf, Split,
  ::testing::Values(
    SplitParamsAndResult{"", ";", {""}},
    SplitParamsAndResult{"test-test", ";", {"test-test"}},
    SplitParamsAndResult{"test;test", ";", {"test", ";", "test"}},
    SplitParamsAndResult{"test-test", "-", {"test", "-", "test"}},
    SplitParamsAndResult{"test est best", " ", {"test", " ", "est", " ", "best"}},
    SplitParamsAndResult{"test::test", "::", {"test", "::", "test"}},
    SplitParamsAndResult{"test-test", "delimiter", {"test-test"}},
    SplitParamsAndResult{"test--est--best", "--", {"test", "--", "est", "--", "best"}},
    SplitParamsAndResult{"--anything--", "--", {"", "--", "anything", "--", ""}},
    SplitParamsAndResult{";", ";", {"", ";", ""}}));
