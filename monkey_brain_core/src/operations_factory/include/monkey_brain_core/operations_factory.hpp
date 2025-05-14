#ifndef MONKEY_BRAIN_CORE_OPERATIONS_FACTORY_HPP
#define MONKEY_BRAIN_CORE_OPERATIONS_FACTORY_HPP

#include "monkey_brain_core/action_description.hpp"
#include "monkey_brain_core/expression_description.hpp"
#include "monkey_brain_core/operator_factory.hpp"

#include "monkey_brain_core/action.hpp"
#include "monkey_brain_core/condition.hpp"
#include "monkey_brain_core/value_types.hpp"

#include <vector>

namespace monkey_brain_core
{

class OperationsFactory
{
public:
  virtual ~OperationsFactory() = default;

  virtual std::vector<std::unique_ptr<Action>> create_actions(
    const ActionDescriptions & descriptions) const = 0;

  virtual ConditionPtr create_condition(
    const ExpressionDescription & description) const = 0;

  virtual void register_operator(OperatorDefinition op) = 0;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_OPERATIONS_FACTORY_HPP
