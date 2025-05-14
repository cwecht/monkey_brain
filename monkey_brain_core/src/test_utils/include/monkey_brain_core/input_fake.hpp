#pragma once

#include "monkey_brain_core/io_plugin.hpp"

namespace monkey_brain_core
{

template<typename Value>
class InputFake : public IOPlugin
{
public:
  InputFake(const Value & value, ValueType type, std::string reference)
  : value_{value}
    , type_{type}
    , reference_{std::move(reference)} {}

  TypedReferences get_references() const final
  {
    return {{reference_, type_, AccessMode::READONLY}};
  }

  void const * get_value_handle(std::string_view const ref) const final
  {
    EXPECT_EQ(ref, reference_);
    return &value_;
  }

  void assign_value(std::string_view const, void const *) final {}

  void perform(std::string_view const) final {}

  std::vector<std::string> get_performance_references() const final {return {};}
  std::vector<std::string> get_events() const final {return {};}

private:
  Value value_;
  ValueType type_;
  std::string reference_;
};

} // namespace monkey_brain_core
