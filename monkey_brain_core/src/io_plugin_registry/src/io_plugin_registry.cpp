#include "monkey_brain_core/io_plugin_registry.hpp"

namespace monkey_brain_core
{

void const * IOPluginRegistry::get_value(std::string_view const ref) const
{
  auto it = input_values_.find(ref);
  return (it == input_values_.end()) ? nullptr : it->second;
}

std::optional<ValueType> IOPluginRegistry::get_type_of(std::string_view const ref) const
{
  auto it = value_types_.find(ref);
  return (it == value_types_.end()) ? std::nullopt :
         std::optional{it->second};
}

void IOPluginRegistry::assign_value(std::string_view const ref, void const * ptr)
{
  auto it = output_values_.find(ref);
  if (it == output_values_.end()) {
    throw std::runtime_error("Could not assign to non-existing reference: " + std::string{ref});
  }
  it->second->assign_value(ref, ptr);
}

void IOPluginRegistry::perform(std::string_view const ref)
{
  auto it = outputters_.find(ref);
  if (it != outputters_.end()) {
    it->second->perform(ref);
  } else if (internal_event_recipients_ != nullptr) {
    internal_event_recipients_->post_internal_event(ref);
  }
}

void IOPluginRegistry::add_plugin(std::unique_ptr<IOPlugin> plugin)
{
  for (const auto & [ref, type, modifier] : plugin->get_references()) {
    if (modifier == AccessMode::READONLY || modifier == AccessMode::READWRITE) {
      input_values_.emplace(ref, plugin->get_value_handle(ref));
    }
    if (modifier == AccessMode::WRITEONLY || modifier == AccessMode::READWRITE) {
      output_values_.emplace(ref, plugin.get());
    }
    value_types_.emplace(ref, type);
  }

  for (const auto & ref : plugin->get_performance_references()) {
    outputters_.emplace(ref, plugin.get());
  }

  plugins_.emplace_back(std::move(plugin));
}

void IOPluginRegistry::add_internal_event_recipient(InternalEventRecipient * recipient)
{
  internal_event_recipients_ = recipient;
}

void IOPluginRegistry::set_event_recipient(EventRecipient * recipient)
{
  for (auto & p : plugins_) {
    p->set_event_recipient(recipient);
  }
}

std::unordered_set<std::string> IOPluginRegistry::get_all_events() const
{
  std::unordered_set<std::string> all_events;
  for (auto & p : plugins_) {
    const auto & events = p->get_events();
    all_events.insert(events.begin(), events.end());
  }
  return all_events;
}

} // namespace monkey_brain_core
