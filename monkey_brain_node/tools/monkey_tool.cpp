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

auto create_operator_plugin_loader()
{
  return create_plugin_loader<monkey_brain_core::OperatorFactory>(
    "monkey_brain_core::OperatorFactory");
}

auto create_decision_engine_plugin_loader()
{
  return create_plugin_loader<monkey_brain_core::DecisionEngineFactory>(
    "monkey_brain_core::DecisionEngineFactory");
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
  const std::size_t n_classes = plugin_loader.getDeclaredClasses().size();
  std::cout << n_classes << " available " << name << ":" << std::endl;
  print_as_list(plugin_loader.getDeclaredClasses());
}

std::string print_typed_references(const monkey_brain_core::TypedReference & d)
{
  return d.reference + " {" + d.type + ", " + to_string(d.access_mode) + ")";
}

int print_io_plugin_list(int argc, const char * argv[])
{
  (void)argc; (void)argv;
  print_io_plugin_list("IOPlugins", create_io_plugin_loader());
  return EXIT_SUCCESS;
}

int print_io_plugin_info(int argc, const char * argv[])
{
  if (argc != 3) {
    std::cout << "usage: monkey_tool show_io_plugin <plugin name>" << std::endl;
    return EXIT_FAILURE;
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
  print_as_list(p->get_references(), print_typed_references);

  std::cout << "perform references: " << p->get_performance_references().size() << std::endl;
  print_as_list(p->get_performance_references());

  std::cout << "events: " << p->get_events().size() << std::endl;
  print_as_list(p->get_events());
  return EXIT_SUCCESS;
}

int print_operator_plugin_list(int argc, const char * argv[])
{
  (void)argc; (void)argv;
  print_io_plugin_list("operators", create_operator_plugin_loader());
  return EXIT_SUCCESS;
}

int print_operator_plugin_info(int argc, const char * argv[])
{
  if (argc != 3) {
    std::cout << "usage: monkey_tool show_operator <plugin name>" << std::endl;
    return EXIT_FAILURE;
  }
  const std::string & plugin_name = argv[2];
  auto plugin_loader = create_operator_plugin_loader();
  const std::string description = plugin_loader.getClassDescription(plugin_name);
  if (not description.empty()) {
    std::cout << description << std::endl;
  } else {
    std::cout << "NO DESCRIPTION AVAILABLE!" << std::endl;
  }
  return EXIT_SUCCESS;
}

int print_decision_engine_plugin_list(int argc, const char * argv[])
{
  (void)argc; (void)argv;
  print_io_plugin_list("decision engines", create_decision_engine_plugin_loader());
  return EXIT_SUCCESS;
}

int list_available_values(int argc, const char * argv[])
{
  if (argc < 3) {
    std::cout << "usage: monkey_tool list_available_values <path_to_plugin_config>" <<
      std::endl;
    return EXIT_FAILURE;
  }

  const std::string config_path = argv[2];
  monkey_brain::PluginLoader plugin_loader;

  const auto io_plugins = monkey_brain::parse_plugin_instances(config_path);
  monkey_brain_core::IOPluginRegistry env;
  for (const auto & instance : io_plugins) {
    env.add_plugin(plugin_loader.create_io_plugin({}, instance));
  }

  print_as_list(env.get_all_values(), print_typed_references);

  return EXIT_SUCCESS;
}

int validate(int argc, const char * argv[])
{
  if (argc < 4) {
    std::cout << "usage: monkey_tool validate <decision_engine> <path_to_plugin_config>" <<
      std::endl;
    return EXIT_FAILURE;
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

  return EXIT_SUCCESS;
}

int usage(int, const char **);

int unknown_command(int argc, const char * argv[])
{
  if (argc > 1) {
    std::cout << "Unknown command: '" << argv[1] << "'" << std::endl;
  }
  usage(argc, argv);
  return EXIT_FAILURE;
}

using CommandFunction = int(*)(int, const char *[]);
struct CommandFunctionWithDescription
{
  CommandFunction command;
  std::string_view description;
};
using CommandToCommandFunctionWithDescription = std::map<std::string_view,
    CommandFunctionWithDescription>;

template<typename Map, typename Value = typename Map::mapped_type>
Value lookup_or_default(const Map & map, const typename Map::key_type & key, const Value & value)
{
  const auto it = map.find(key);
  return (it == map.end()) ? value : it->second;
}

const CommandToCommandFunctionWithDescription command_to_function {
  {"list_io_plugins", {&print_io_plugin_list, "Display a list of all available io plugins."}},
  {"list_operator_plugins",
    {&print_operator_plugin_list, "Display a list of all available operator plugins."}},
  {"list_decision_engine_plugins",
    {&print_decision_engine_plugin_list,
      "Display a list of all available decision engine plugins."}},
  {"show_io_plugin", {&print_io_plugin_info, "Display parameters and references of an io plugin."}},
  {"show_operator_plugin",
    {&print_operator_plugin_info, "Display description of an operator plugin."}},
  {"list_available_values",
    {&list_available_values, "List all available value for given IO-Plugins configuration."}},
  {"validate", {&validate, "Validates a given configuration."}},
  {"help", {&usage, "Print this help message."}}
};

size_t get_max_command_length()
{
  return std::max_element(command_to_function.begin(), command_to_function.end(),
           [] (const auto & lhs, const auto & rhs) {
             return lhs.first.size() < rhs.first.size();
                          })->first.size();
}

int usage(int, const char **)
{
  const std::size_t width = get_max_command_length();
  std::cout << "usage: monkey_tool <command>" << '\n';
  std::cout << '\n';
  std::cout << "monkey_tool is a helper tool for using monkey brain." << '\n';
  std::cout << '\n';
  std::cout << "Commands:" << '\n';
  std::cout << std::left;
  for (const auto & [command, fun_d] : command_to_function) {
    std::cout << "  " << std::setw(width) << command << " " << fun_d.description << '\n';
  }
  std::cout << std::flush;
  return EXIT_SUCCESS;
}

int main(int argc, const char * argv[])
{
  const std::string_view command = argc > 1 ? argv[1] : "";
  const CommandFunctionWithDescription DEFAULT{&unknown_command, ""};
  try {
    return lookup_or_default(command_to_function, command, DEFAULT).command(argc, argv);
  } catch (const std::exception & ex) {
    std::cerr << "ERROR: " << ex.what() << std::endl;
    return EXIT_FAILURE;
  }
}
