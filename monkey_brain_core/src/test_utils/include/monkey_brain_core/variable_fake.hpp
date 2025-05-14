#pragma once

#include "monkey_brain_core/io_plugin.hpp"

namespace monkey_brain_core
{

template<typename Value>
class VariableFake : public IOPlugin
{
  static_assert(std::is_trivial_v<Value>);

public:
  VariableFake(const Value & value, ValueType type, std::string reference)
  : mValue{value}
    , mType{type}
    , mReference{std::move(reference)} {}

  TypedReferences get_references() const final
  {
    return {{mReference, mType, AccessMode::READWRITE}};
  }

  void const * get_value_handle(std::string_view const) const final
  {
    return &mValue;
  }

  void assign_value(std::string_view const, void const * ptr) final
  {
    std::memcpy(&mValue, ptr, sizeof(mValue));
  }

  void perform(std::string_view const) final {}

  std::vector<std::string> get_performance_references() const final {return {};}
  std::vector<std::string> get_events() const final {return {};}

private:
  Value mValue;
  ValueType mType;
  std::string mReference;
};

} // namespace monkey_brain_core
