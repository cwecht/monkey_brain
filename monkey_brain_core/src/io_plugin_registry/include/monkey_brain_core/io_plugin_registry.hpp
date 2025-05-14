#ifndef MONKEY_BRAIN_CORE_IO_PLUGIN_REGISTRY_HPP
#define MONKEY_BRAIN_CORE_IO_PLUGIN_REGISTRY_HPP

#include "monkey_brain_core/environment.hpp"
#include "monkey_brain_core/event_recipient.hpp"
#include "monkey_brain_core/io_plugin.hpp"

#include <map>
#include <memory>
#include <string_view>
#include <unordered_set>

namespace monkey_brain_core
{

class IOPluginRegistry : public Environment
{
public:
  void assign_value(std::string_view const ref, void const * ptr) override;
  void perform(std::string_view const ref) override;

  std::optional<ValueType> get_type_of(std::string_view const ref) const override;

  void add_internal_event_recipient(InternalEventRecipient * recipient) override;
  void set_event_recipient(EventRecipient * recipient) override;

  std::unordered_set<std::string> get_all_events() const override;

  void add_plugin(std::unique_ptr<IOPlugin> outputter);

protected:
  void const * get_value(std::string_view const ref) const override;

private:
  std::vector<std::unique_ptr<IOPlugin>> plugins_;
  std::map<std::string, IOPlugin *, std::less<>> outputters_;
  std::map<std::string, void const *, std::less<>> input_values_;
  std::map<std::string, IOPlugin *, std::less<>> output_values_;
  std::map<std::string, ValueType, std::less<>> value_types_;
  InternalEventRecipient * internal_event_recipients_;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_IO_PLUGIN_REGISTRY_HPP
