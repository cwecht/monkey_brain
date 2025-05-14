#ifndef MONKEY_BRAIN_NODE_HPP
#define MONKEY_BRAIN_NODE_HPP

#include "monkey_brain_core/io_plugin_registry.hpp"
#include "monkey_brain_core/decision_engine.hpp"

#include "monkey_brain/plugins_loader.hpp"

#include "rclcpp/rclcpp.hpp"

namespace monkey_brain
{

class MonkeyBrainNode : public rclcpp::Node
{
public:
  MonkeyBrainNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{});

private:
  PluginLoader plugin_loader_;
  std::unique_ptr<monkey_brain_core::IOPluginRegistry> env_;
  std::unique_ptr<monkey_brain_core::DecisionEngine> descision_engine_;

  static std::unique_ptr<monkey_brain_core::IOPluginRegistry> make_environment(
    rclcpp::Node * node,
    PluginLoader & plugin_loader);
};

} // namespace monkey_brain
#endif // MONKEY_BRAIN_NODE_HPP
