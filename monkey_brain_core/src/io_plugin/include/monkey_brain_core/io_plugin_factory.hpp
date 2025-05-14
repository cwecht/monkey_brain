#ifndef MONKEY_BRAIN_CORE_IO_PLUGING_FACTORY_HPP
#define MONKEY_BRAIN_CORE_IO_PLUGING_FACTORY_HPP

#include "monkey_brain_core/io_plugin.hpp"

#include <yaml-cpp/yaml.h>

#include <any>
#include <memory>
#include <string>

namespace monkey_brain_core
{

class IOPluginFactory
{
public:
  virtual ~IOPluginFactory() = default;
  virtual std::unique_ptr<monkey_brain_core::IOPlugin> instantiate(
    std::any node,
    const std::string & topic,
    const YAML::Node & params) = 0;
};

template<typename IOPluginImpl>
class IOPluginFactoryImpl : public IOPluginFactory
{
public:
  std::unique_ptr<monkey_brain_core::IOPlugin> instantiate(
    std::any node,
    const std::string & topic,
    const YAML::Node & params) final
  {
    return std::make_unique<IOPluginImpl>(std::move(node), topic, params);
  }
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_IO_PLUGING_FACTORY_HPP
