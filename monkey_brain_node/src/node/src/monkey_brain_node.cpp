#include "monkey_brain/monkey_brain_node.hpp"

#include "monkey_brain_core/concrete_operations_factory.hpp"
#include "monkey_brain_core/decision_engine.hpp"
#include "monkey_brain_core/io_plugin_factory.hpp"
#include "monkey_brain_core/operator_factory.hpp"

#include "monkey_brain/plugins_parser.hpp"

#include "monkey_brain_ros_utils/ros_context.hpp"

#include <pluginlib/class_loader.hpp>

namespace monkey_brain
{
std::unique_ptr<monkey_brain_core::IOPluginRegistry>
MonkeyBrainNode::make_environment(rclcpp::Node * node, PluginLoader & plugin_loader)
{
  auto env = std::make_unique<monkey_brain_core::IOPluginRegistry>();
  const std::string path_to_ios =
    node->declare_parameter<std::string>("path_to_ios");

  auto io_plugins = parse_plugin_instances(path_to_ios);
  for (const auto & instance : io_plugins) {
    env->add_plugin(plugin_loader.create_io_plugin(node, instance));
  }

  return env;
}

namespace
{

monkey_brain_core::ConreteOperationsFactory make_operations_factory(
  rclcpp::Node * node,
  monkey_brain_core::Environment & env, PluginLoader & plugin_loader)
{
  const std::string path_to_ios =
    node->get_parameter("path_to_ios").get_value<std::string>();

  auto op_plugins = parse_operator_plugin_instances(path_to_ios);
  monkey_brain_core::ConreteOperationsFactory concrete_factory{env};
  for (const auto & instance : op_plugins) {
    for (auto && def : plugin_loader.create_operation(instance)) {
      concrete_factory.register_operator(std::move(def));
    }
  }

  return concrete_factory;
}

std::unique_ptr<monkey_brain_core::DecisionEngine>
make_descision_engine(
  rclcpp::Node * node,
  std::function<void()> on_shutdown,
  monkey_brain_core::Environment & env, PluginLoader & plugin_loader)
{
  monkey_brain_core::ConreteOperationsFactory ops_factory{make_operations_factory(node, env,
          plugin_loader)};
  const std::string path_to_ios =
    node->get_parameter("path_to_ios").get_value<std::string>();

  monkey_brain_ros_utils::ROSContext context{node};
  return plugin_loader.create_decision_engine("scxml_state_machine", context,
        std::move(on_shutdown), ops_factory, env);
}

} // namespace

MonkeyBrainNode::MonkeyBrainNode(const rclcpp::NodeOptions & node_options)
: Node("monkey_brain", node_options)
  , env_{make_environment(this, plugin_loader_)}
  , descision_engine_{make_descision_engine(this, [] {
        rclcpp::shutdown();
      }, *env_, plugin_loader_)} {}

} // namespace monkey_brain
