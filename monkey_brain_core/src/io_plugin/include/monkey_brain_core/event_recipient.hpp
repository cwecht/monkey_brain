#ifndef MONKEY_BRAIN_CORE_EVENT_RECIPIENT_HPP
#define MONKEY_BRAIN_CORE_EVENT_RECIPIENT_HPP

#include <string_view>

namespace monkey_brain_core
{

class EventRecipient
{
public:
  virtual ~EventRecipient() = default;
  virtual void post_event(const std::string_view) = 0;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_EVENT_RECIPIENT_HPP
