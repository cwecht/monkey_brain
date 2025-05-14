#include "monkey_brain/plugins_loader.hpp"

namespace monkey_brain
{

std::unique_ptr<monkey_brain_core::IOPlugin>
PluginLoader::create_io_plugin(std::any node, const PluginInstance & instance)
{
  if (io_plugin_loader_ == nullptr) {
    io_plugin_loader_ =
      std::make_unique<pluginlib::ClassLoader<monkey_brain_core::IOPluginFactory>>(
      "monkey_brain", "monkey_brain_core::IOPluginFactory");
  }
  auto factory = io_plugin_loader_->createUniqueInstance(instance.name);
  return factory->instantiate(node, instance.topic, instance.params);
}

monkey_brain_core::OperatorDefinitions
PluginLoader::create_operation(const std::string & instance)
{
  if (operator_plugin_loader_ == nullptr) {
    operator_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<monkey_brain_core::OperatorFactory>>(
      "monkey_brain", "monkey_brain_core::OperatorFactory");
  }
  auto factory = operator_plugin_loader_->createUniqueInstance(instance);
  return factory->create_operation();
}

std::unique_ptr<monkey_brain_core::DecisionEngine>
PluginLoader::create_decision_engine(
  const std::string & instance,
  monkey_brain_core::Context & context,
  std::function<void()> on_shutdown,
  monkey_brain_core::OperationsFactory & ops_factory,
  monkey_brain_core::Environment & env)
{
  if (decision_engine_plugin_loader_ == nullptr) {
    decision_engine_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<monkey_brain_core::DecisionEngineFactory>>(
    "monkey_brain", "monkey_brain_core::DecisionEngineFactory");
  }
  auto factory = decision_engine_plugin_loader_->createUniqueInstance(instance);
  return factory->create(&context, std::move(on_shutdown), ops_factory, env);
}

} // namespace monkey_brain
