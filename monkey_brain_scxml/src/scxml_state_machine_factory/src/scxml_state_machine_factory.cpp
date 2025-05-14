#include "monkey_brain_scxml/scxml_state_machine_factory.hpp"

#include "monkey_brain_scxml/build_states.hpp"
#include "monkey_brain_scxml/in_state_operator.hpp"
#include "monkey_brain_scxml/parse_scxml.hpp"
#include "monkey_brain_scxml/state_machine.hpp"
#include "monkey_brain_scxml/validate_state_machine.hpp"

#include "veneer/veneer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

#include <filesystem>

namespace monkey_brain_scxml
{
namespace
{

std::unique_ptr<StateMachineObserver> create_sm_observer(monkey_brain_core::Context * context)
{
  const std::string name_of_observer =
    context->declare_parameter("state_machine_observer", std::string{""});

  if (name_of_observer == "") {
    return nullptr;
  }

  static std::unique_ptr<pluginlib::ClassLoader<StateMachineObserverFactory>>
  plugin_loader = nullptr;
  plugin_loader =
    std::make_unique<pluginlib::ClassLoader<StateMachineObserverFactory>>(
    "monkey_brain", "StateMachineObserverFactory");
  return plugin_loader->createUniqueInstance(name_of_observer)->create(context);
}

void print_validation_errors(const ValidationErrors & errors, const veneer::LoggerPtr & logger)
{
  for (const auto & error : errors) {
    VENEER_LOG_WARN(logger, error.message.c_str());
  }
}

} // namespace

std::unique_ptr<monkey_brain_core::DecisionEngine>
SCXMLStateMachineFactory::create(
  monkey_brain_core::Context * context,
  std::function<void()> on_shutdown,
  monkey_brain_core::OperationsFactory & ops_factory,
  monkey_brain_core::Environment & env) const
{
  auto logger = context->get_logger()->get_child("scxml_state_machine_factory");
  const std::filesystem::path path_to_scxml =
    context->declare_parameter<std::string>("path_to_scxml");

  auto sm = std::make_unique<StateMachine>(std::move(on_shutdown), create_sm_observer(context),
      logger);
  ops_factory.register_operator(create_in_state_operator(sm.get()));

  const auto state_machine_description = parse_scxml(path_to_scxml);
  print_validation_errors(validate_state_machine(state_machine_description, env), logger);

  auto states = build_states(state_machine_description, ops_factory);
  env.add_internal_event_recipient(sm.get());
  env.set_event_recipient(sm.get());
  if (!context->is_dry_run()) {
    sm->initialize(std::move(states));
  }
  return sm;
}

} // namespace monkey_brain_scxml

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_scxml::SCXMLStateMachineFactory,
  monkey_brain_core::DecisionEngineFactory)
