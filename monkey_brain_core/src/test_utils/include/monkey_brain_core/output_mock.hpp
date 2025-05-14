#pragma once

namespace monkey_brain_core
{

struct OutputPluginMock : IOPlugin
{
  OutputPluginMock(ValueType type, std::string pReference, std::string pPerforreference_)
  : type_{type}
    , reference_{std::move(pReference)}
    , performance_reference_{std::move(pPerforreference_)} {}

  TypedReferences get_references() const final
  {
    return {{reference_, type_, AccessMode::WRITEONLY}};
  }

  void const * get_value_handle(std::string_view const) const final
  {
    return nullptr;
  }

  MOCK_METHOD2(assign_value, void(std::string_view const, void const *));

  MOCK_METHOD1(perform, void(std::string_view const));
  std::vector<std::string> get_performance_references() const final
  {
    return {performance_reference_};
  }

  MOCK_CONST_METHOD0(get_events, std::vector<std::string>());

  ValueType type_;
  std::string reference_;
  std::string performance_reference_;
};

} // namespace monkey_brain_core
