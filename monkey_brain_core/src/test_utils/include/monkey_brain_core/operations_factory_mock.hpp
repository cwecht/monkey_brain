#pragma once

#include "monkey_brain_core/operations_factory.hpp"

#include <gmock/gmock.h>

namespace monkey_brain_core
{

class OperationsFactoryMock : public OperationsFactory
{
public:
  OperationsFactoryMock()
  {
    ON_CALL(
      *this,
      create_actions).WillByDefault(
      [](const ActionDescriptions &) {
        return std::vector<std::unique_ptr<Action>>{};
      });
  }

  MOCK_CONST_METHOD1(
    create_actions,
    std::vector<std::unique_ptr<Action>>(const ActionDescriptions &));

  MOCK_CONST_METHOD1(
    create_condition,
    ConditionPtr(const ExpressionDescription &));

  MOCK_METHOD1(register_operator, void(OperatorDefinition));
};

} // namespace monkey_brain_core
