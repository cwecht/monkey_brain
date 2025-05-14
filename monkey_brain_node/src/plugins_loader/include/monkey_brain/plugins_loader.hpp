#ifndef MONKEY_BRAIN_PLUGINS_LOADER_HPP
#define MONKEY_BRAIN_PLUGINS_LOADER_HPP

#include "monkey_brain/plugins_parser.hpp"

#include "monkey_brain_core/decision_engine.hpp"
#include <monkey_brain_core/io_plugin.hpp>
#include "monkey_brain_core/io_plugin_factory.hpp"
#include "monkey_brain_core/operator_factory.hpp"

#include "rclcpp/node.hpp"

#include <pluginlib/class_loader.hpp>

#include <any>
#include <memory>
#include <string>

namespace monkey_brain
{

class PluginLoader {
public:
  std::unique_ptr<monkey_brain_core::IOPlugin> create_io_plugin(
    std::any node,
    const PluginInstance & instance);
  monkey_brain_core::OperatorDefinitions create_operation(const std::string & instance);
  std::unique_ptr<monkey_brain_core::DecisionEngine> create_decision_engine(
    const std::string & instance,
    monkey_brain_core::Context & context,
    std::function<void()> on_shutdown,
    monkey_brain_core::OperationsFactory & ops_factory,
    monkey_brain_core::Environment & env);

private:
  std::unique_ptr<pluginlib::ClassLoader<monkey_brain_core::IOPluginFactory>> io_plugin_loader_ =
    nullptr;
  std::unique_ptr<pluginlib::ClassLoader<monkey_brain_core::OperatorFactory>>
  operator_plugin_loader_ = nullptr;
  std::unique_ptr<pluginlib::ClassLoader<monkey_brain_core::DecisionEngineFactory>>
  decision_engine_plugin_loader_ = nullptr;
};

} // namespace monkey_brain
#endif // MONKEY_BRAIN_PLUGINS_LOADER_HPP
