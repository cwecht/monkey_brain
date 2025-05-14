#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/concrete_operations_factory.hpp"
#include "monkey_brain_core/io_plugin_registry.hpp"
#include "monkey_brain_scxml/state_machine.hpp"
#include "monkey_brain_scxml/scxml_state_machine_factory.hpp"

#include "monkey_brain_core/variable_fake.hpp"
#include "monkey_brain_core/context.hpp"

#include <filesystem>

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

const auto kTestDataFolder{getenv<std::filesystem::path>("SCXML_IRP_TEST_DATA_FOLDER")};

} // namespace

class MockContext : public monkey_brain_core::Context
{
public:
  MOCK_CONST_METHOD1(holds, bool(const std::type_info &));
  MOCK_CONST_METHOD0(get_concrete_context_impl, void * ());
  MOCK_METHOD3(
    declare_parameter_impl,
    const void * (const std::string &, void const *, const std::type_info &));
  MOCK_CONST_METHOD0(is_dry_run, bool());

  MOCK_CONST_METHOD0(get_logger, veneer::LoggerPtr());
};

using testing::ElementsAre;
using testing::Return;
using testing::_;

namespace mbc = monkey_brain_core;

using ASCXMLIRPStateMachine = ::testing::TestWithParam<std::string>;

TEST_P(ASCXMLIRPStateMachine, ends_in_conf_pass) {
  auto env = std::make_unique<mbc::IOPluginRegistry>();
  env->add_plugin(
    std::make_unique<mbc::VariableFake<std::uint32_t>>(
      0u, mbc::ValueTypes::INT32, "val1"));
  env->add_plugin(
    std::make_unique<mbc::VariableFake<std::uint32_t>>(
      0u, mbc::ValueTypes::INT32, "val2"));
  env->add_plugin(
    std::make_unique<mbc::VariableFake<std::uint32_t>>(
      0u, mbc::ValueTypes::INT32, "val3"));
  env->add_plugin(
    std::make_unique<mbc::VariableFake<std::uint32_t>>(
      0u, mbc::ValueTypes::INT32, "val4"));
  env->add_plugin(
    std::make_unique<mbc::VariableFake<std::uint32_t>>(
      0u, mbc::ValueTypes::INT32, "val5"));

  mbc::ConreteOperationsFactory ops_factory(*env);

  const std::string path_to_scxml = (kTestDataFolder / GetParam()).string();
  MockContext context;
  EXPECT_CALL(context, is_dry_run).WillRepeatedly(Return(false));
  EXPECT_CALL(context,
    declare_parameter_impl("path_to_scxml", _, _)).WillRepeatedly(Return(&path_to_scxml));
  const std::string no_observer;
  EXPECT_CALL(
    context,
    declare_parameter_impl("state_machine_observer", _, _)).WillRepeatedly(Return(&no_observer));
  EXPECT_CALL(context, get_logger).WillRepeatedly([] {
      return veneer::ZeroLogger::Factory{}.get_logger("");
                                                                                                          });

  auto de = SCXMLStateMachineFactory{}.create(&context, {}, ops_factory, *env);
  auto * const sm = dynamic_cast<StateMachine *>(de.get());
  ASSERT_NE(sm, nullptr);
  ASSERT_THAT(sm->active_states(), ElementsAre("conf:pass"));
}

namespace
{
std::string fix_test_name(
  const testing::TestParamInfo<ASCXMLIRPStateMachine::ParamType> & info)
{
  std::string msg = info.param.substr(0, info.param.find(".scxml"));
  std::replace_if(msg.begin(), msg.end(), [](char c) {return !std::isalnum(c);}, '_');
  return msg;
}
} // namespace

INSTANTIATE_TEST_SUITE_P(
  AllOf, ASCXMLIRPStateMachine,
  ::testing::Values(
    "test144.scxml",
    "test310.scxml",
    "test355.scxml",
    "test364.scxml",
    "test372.scxml",
    "test375.scxml",
    "test387.scxml",
    "test388.scxml",
    "test570.scxml",
    "test576.scxml",
    "test579.scxml",
    "test580.scxml",
    "test403a.scxml",
    "test403b.scxml",
    "test403c.scxml",
    "test404.scxml",
    "test405.scxml",
    "test406.scxml",
    "test407.scxml",
    "test412.scxml",
    "test413.scxml",
    "test416.scxml",
    "test417.scxml",
    "test419.scxml",
    "test503.scxml",
    "test504.scxml",
    "test505.scxml",
    "test506.scxml",
    "test533.scxml"
  ),
  fix_test_name
);
