#include "monkey_brain_core/cli_context.hpp"
#include "monkey_brain_core/concrete_operations_factory.hpp"
#include "monkey_brain_core/io_plugin_factory.hpp"
#include "monkey_brain_core/io_plugin_registry.hpp"

#include "monkey_brain/plugins_loader.hpp"
#include "monkey_brain/plugins_parser.hpp"

#include <pluginlib/class_loader.hpp>

#include <iostream>
#include <string_view>
#include <unordered_map>

using monkey_brain_core::AccessMode;

std::string to_string(AccessMode mode)
{
  switch (mode) {
    case AccessMode::READONLY: return "R";
    case AccessMode::WRITEONLY: return "W";
    case AccessMode::READWRITE: return "R/W";
  }
  return "";
}

template<typename T>
auto create_plugin_loader(const char * type)
{
  try {
    return pluginlib::ClassLoader<T>("monkey_brain", type);
  } catch(...) {
    std::cout << "no plugins found! Did you source your workspace?" << std::endl;
    std::exit(-1);
  }
}

auto create_io_plugin_loader()
{
  return create_plugin_loader<monkey_brain_core::IOPluginFactory>(
    "monkey_brain_core::IOPluginFactory");
}

struct identity
{
  template<typename T>
  const T & operator()(const T & x) const noexcept {return x;}
};

template<typename Rng, typename Fun = identity>
void print_as_list(const Rng & rng, Fun fun = identity{})
{
  for (const auto & plugin : rng) {
    std::cout << " - " << fun(plugin) << std::endl;
  }
}

template<typename PlugingLoader>
void print_io_plugin_list(std::string_view name, PlugingLoader plugin_loader)
{
  std::cout << plugin_loader.getDeclaredClasses().size() << " available " << name << ":" <<
    std::endl;
  print_as_list(plugin_loader.getDeclaredClasses());
}

int print_io_plugin_list(int argc, const char * argv[])
{
  (void)argc; (void)argv;
  print_io_plugin_list("IOPlugins", create_io_plugin_loader());
  return 0;
}

int print_io_plugin_info(int argc, const char * argv[])
{
  if (argc != 3) {
    std::cout << "usage: monkey_tool show_io_plugin <plugin name>" << std::endl;
    return -1;
  }
  const std::string & plugin_name = argv[2];
  auto plugin_loader = create_io_plugin_loader();
  auto p_factory = plugin_loader.createUniqueInstance(plugin_name);
  assert(p_factory != nullptr);
  auto p = p_factory->instantiate({}, "<prefix>", {});
  assert(p != nullptr);
  const std::string description = plugin_loader.getClassDescription(plugin_name);
  if (not description.empty()) {
    std::cout << description << std::endl;
  }
  std::cout << "value references: " << p->get_references().size() << std::endl;
  print_as_list(p->get_references(),
    [] (const auto & d) {
      return d.reference + " {" + d.type + ", " + to_string(d.access_mode) + ")";
                });

  std::cout << "perform references: " << p->get_performance_references().size() << std::endl;
  print_as_list(p->get_performance_references());

  std::cout << "events: " << p->get_events().size() << std::endl;
  print_as_list(p->get_events());
  return 0;
}

auto create_operator_plugin_loader()
{
  return create_plugin_loader<monkey_brain_core::OperatorFactory>(
    "monkey_brain_core::OperatorFactory");
}

int print_operator_plugin_list(int argc, const char * argv[])
{
  (void)argc; (void)argv;
  print_io_plugin_list("operators", create_operator_plugin_loader());
  return 0;
}

int print_operator_plugin_info(int argc, const char * argv[])
{
  if (argc != 3) {
    std::cout << "usage: monkey_tool show_operator <plugin name>" << std::endl;
    return -1;
  }
  const std::string & plugin_name = argv[2];
  auto plugin_loader = create_operator_plugin_loader();
  const std::string description = plugin_loader.getClassDescription(plugin_name);
  if (not description.empty()) {
    std::cout << description << std::endl;
  } else {
    std::cout << "NO DESCRIPTION AVAILABLE!" << std::endl;
  }
  return 0;
}
int validate(int argc, const char * argv[])
{
  if (argc < 4) {
    std::cout << "usage: monkey_tool validate <decision_engine> <path_to_plugin_config>" <<
      std::endl;
    return -1;
  }
  const std::string decision_engine = argv[2];
  const std::string config_path = argv[3];
  monkey_brain::PluginLoader plugin_loader;

  const auto io_plugins = monkey_brain::parse_plugin_instances(config_path);
  monkey_brain_core::IOPluginRegistry env;
  for (const auto & instance : io_plugins) {
    env.add_plugin(plugin_loader.create_io_plugin({}, instance));
  }

  const auto op_plugins = monkey_brain::parse_operator_plugin_instances(config_path);
  monkey_brain_core::ConreteOperationsFactory concrete_factory{env};
  for (const auto & instance : op_plugins) {
    for (auto && def : plugin_loader.create_operation(instance)) {
      concrete_factory.register_operator(std::move(def));
    }
  }

  monkey_brain_core::CLIContext context{argc, argv};
  const auto de = plugin_loader.create_decision_engine(decision_engine, context, {},
    concrete_factory, env);

  return 0;
}

int usage(int, const char **)
{
  std::cout << "usage: monkey_tool <command>" << std::endl;
  std::cout << std::endl;
  std::cout << "monkey_tool is a helper tool for using monkey brain." << std::endl;
  std::cout << std::endl;
  std::cout << "Commands:" << std::endl;
  std::cout << "  list_io_plugins       Display a list of all available io plugins." << std::endl;
  std::cout << "  list_operator_plugins Display a list of all available operator plugins." <<
    std::endl;
  std::cout << "  show_io_plugin        Display parameters and references of an io plugin." <<
    std::endl;
  std::cout << "  show_operator_plugin  Display description of an operator plugin." << std::endl;
  std::cout << "  validate              Validates a given configuration." << std::endl;
  return 0;
}

int unknown_command(int argc, const char * argv[])
{
  assert(argc > 0);
  std::cout << "Unknown command: '" << argv[1] << std::endl;
  usage(argc, argv);
  return -1;
}

using Command = int(*)(int, const char *[]);
using CommandToFunction = std::unordered_map<std::string_view, Command>;

template<typename Map, typename Key, typename Value>
Value lookup_or_default(const Map & map, const Key & key, const Value & value)
{
  const auto it = map.find(key);
  return (it == map.end()) ? value : it->second;
}

int main(int argc, const char * argv[])
{
  const CommandToFunction command_to_function {
    {"list_io_plugins", &print_io_plugin_list},
    {"list_operator_plugins", &print_operator_plugin_list},
    {"show_io_plugin", &print_io_plugin_info},
    {"show_operator_plugin", &print_operator_plugin_info},
    {"validate", &validate},
    {"help", &usage},
    {"", &usage}
  };

  const std::string_view command = argc > 1 ? argv[1] : "";
  return lookup_or_default(command_to_function, command, &unknown_command)(argc, argv);
}
