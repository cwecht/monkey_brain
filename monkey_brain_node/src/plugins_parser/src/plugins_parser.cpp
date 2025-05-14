#include "monkey_brain/plugins_parser.hpp"

namespace monkey_brain
{
namespace
{

PluginInstances parse_instances(const YAML::Node & instances)
{
  PluginInstances pis;
  pis.reserve(instances.size());
  for (auto const & elem : instances) {
    pis.push_back(
      PluginInstance{elem["name"].as<std::string>(),
        elem["topic"].as<std::string>(), elem["params"]});
  }
  return pis;
}

std::vector<std::string> parse_operator_instances(const YAML::Node & node)
{
  std::vector<std::string> ops;
  ops.reserve(node.size());
  for (auto const & elem : node) {
    ops.push_back(elem.as<std::string>());
  }
  return ops;
}

} // namespace

PluginInstances parse_plugin_instances(const std::string path)
{
  YAML::Node config = YAML::LoadFile(path);
  return parse_instances(config["io"]);
}

std::vector<std::string> parse_operator_plugin_instances(const std::string path)
{
  YAML::Node config = YAML::LoadFile(path);
  return parse_operator_instances(config["operators"]);
}

} // namespace monkey_brain
