#ifndef MONKEY_BRAIN_PLUGINS_PARSER_HPP
#define MONKEY_BRAIN_PLUGINS_PARSER_HPP

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace monkey_brain
{

struct PluginInstance
{
  std::string name;
  std::string topic;
  const YAML::Node params;
};

struct DecisionEngineInstance
{
  std::string name;
  const YAML::Node params;
};

using PluginInstances = std::vector<PluginInstance>;

PluginInstances parse_plugin_instances(const std::string path);

std::vector<std::string> parse_operator_plugin_instances(const std::string path);

} // namespace monkey_brain
#endif // MONKEY_BRAIN_PLUGINS_PARSER_HPP
