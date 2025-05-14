#ifndef MONKEY_BRAIN_CORE_IO_PLUGIN_HPP
#define MONKEY_BRAIN_CORE_IO_PLUGIN_HPP

#include "monkey_brain_core/event_recipient.hpp"
#include "monkey_brain_core/value_types.hpp"

#include <cassert>
#include <string>
#include <string_view>
#include <vector>

#include <iostream>

namespace monkey_brain_core
{

class IOPlugin
{
public:
  virtual ~IOPlugin() = default;

  virtual TypedReferences get_references() const = 0;
  virtual void const * get_value_handle(std::string_view const ref) const = 0;
  virtual void assign_value(std::string_view const ref, void const * ptr) = 0;

  virtual void perform(std::string_view const ref) = 0;
  virtual std::vector<std::string> get_performance_references() const = 0;
  virtual std::vector<std::string> get_events() const = 0;

  void set_event_recipient(EventRecipient * event_recipient)
  {
    assert(event_recipient != nullptr);
    event_recipient_ = event_recipient;
  }

  void post_event(const std::string_view event)
  {
    assert(event_recipient_ != nullptr);
    event_recipient_->post_event(event);
  }

private:
  EventRecipient * event_recipient_ = nullptr;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_IO_PLUGIN_HPP
