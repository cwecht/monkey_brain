#ifndef MONKEY_BRAIN_CORE_ENVIRONMENT_HPP
#define MONKEY_BRAIN_CORE_ENVIRONMENT_HPP

#include "monkey_brain_core/event_recipient.hpp"
#include "monkey_brain_core/value_types.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string_view>
#include <unordered_set>

namespace monkey_brain_core
{

class InternalEventRecipient
{
public:
  virtual ~InternalEventRecipient() = default;
  virtual void post_internal_event(std::string_view const ref) = 0;
};

class Environment
{
public:
  Environment() = default;
  virtual ~Environment() = default;
  Environment(const Environment &) = delete;
  Environment & operator=(const Environment &) & = delete;
  Environment(Environment &&) = delete;
  Environment & operator=(Environment &&) & = delete;

  template<typename T>
  T const * get_value(std::string_view const ref) const
  {
    return reinterpret_cast<T const *>(get_value(ref));
  }

  virtual void assign_value(std::string_view const ref, void const * ptr) = 0;
  virtual void perform(std::string_view const ref) = 0;

  virtual std::optional<ValueType> get_type_of(std::string_view const ref) const = 0;

  virtual void add_internal_event_recipient(InternalEventRecipient * recipient) = 0;
  virtual void set_event_recipient(EventRecipient * recipient) = 0;

  virtual std::unordered_set<std::string> get_all_events() const = 0;

protected:
  virtual void const * get_value(std::string_view const ref) const = 0;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_ENVIRONMENT_HPP
