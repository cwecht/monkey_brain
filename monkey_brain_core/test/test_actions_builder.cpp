#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/build_actions.hpp"
#include "monkey_brain_core/io_plugin_registry.hpp"

#include "monkey_brain_core/expression_description_helper.hpp"
#include "monkey_brain_core/input_fake.hpp"
#include "monkey_brain_core/output_mock.hpp"

namespace mbc = monkey_brain_core;

const std::string kReference{"reference/to/out"};
const std::string kPerformReference{"reference/perform"};
const std::string kInt32Reference{"reference/to/int32"};
const std::int32_t kInt32Value = 42;

struct ActionTestCase
{
  mbc::ActionDescriptions const descriptions;
  int const expected_performs;
  int const expected_assigns;
};

class BuildAction : public ::testing::TestWithParam<ActionTestCase>
{
public:
  BuildAction()
  {
    auto outputter = std::make_unique<mbc::OutputPluginMock>(
      mbc::ValueTypes::INT32, kReference,
      kPerformReference);
    outputter_mock = outputter.get();
    env.add_plugin(std::move(outputter));
    env.add_plugin(
      std::make_unique<mbc::InputFake<std::int32_t>>(
        kInt32Value, mbc::ValueTypes::INT32, kInt32Reference));
  }

protected:
  mbc::IOPluginRegistry env;
  mbc::OutputPluginMock * outputter_mock;
};

TEST_P(BuildAction, builds_working_condition_with_perform) {
  auto const & descriptions = GetParam().descriptions;
  auto const & expected_performs = GetParam().expected_performs;
  auto const & expected_assigns = GetParam().expected_assigns;
  auto actions = mbc::build_actions(descriptions, env);
  EXPECT_CALL(*outputter_mock, perform).Times(expected_performs);
  EXPECT_CALL(*outputter_mock, assign_value).Times(expected_assigns);
  mbc::execute(actions);
}

const mbc::ActionDescriptions kSimplePerformAction{mbc::ActionDescription{"PERFORM",
    {REF(kPerformReference)}}};
const mbc::ActionDescriptions kSimpleAssignAction{mbc::ActionDescription{":=",
    {REF(kReference), REF(kInt32Reference)}}};
const mbc::ActionDescriptions kDoublePerformAction{kSimplePerformAction.front(),
  kSimplePerformAction.front()};
const mbc::ActionDescriptions kDoubleAssignmAction{kSimpleAssignAction.front(),
  kSimpleAssignAction.front()};
const mbc::ActionDescriptions kOnePerformOneAssignAction{kSimpleAssignAction.front(),
  kSimplePerformAction.front()};

INSTANTIATE_TEST_SUITE_P(
  AllOf, BuildAction,
  ::testing::Values(
    ActionTestCase{kSimplePerformAction, 1, 0},
    ActionTestCase{kDoublePerformAction, 2, 0},
    ActionTestCase{kDoubleAssignmAction, 0, 2},
    ActionTestCase{kOnePerformOneAssignAction, 1, 1}));
