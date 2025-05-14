#pragma once

#include "monkey_brain_core/environment.hpp"

#include <gmock/gmock.h>

namespace monkey_brain_core
{

struct EnvironmentMock : public Environment
{
  MOCK_METHOD2(assign_value, void(std::string_view const, void const *));
  MOCK_METHOD1(perform, void(std::string_view const));

  MOCK_CONST_METHOD1(get_type_of, std::optional<ValueType>(std::string_view const));

  MOCK_METHOD1(add_internal_event_recipient, void(InternalEventRecipient *));
  MOCK_METHOD1(set_event_recipient, void(EventRecipient *));

  MOCK_CONST_METHOD0(get_all_events, std::unordered_set<std::string>());

protected:
  MOCK_CONST_METHOD1(get_value, void const * (std::string_view const));
};

} // namespace monkey_brain_core
