#include "monkey_brain_core/concrete_operations_factory.hpp"

#include "monkey_brain_core/build_actions.hpp"
#include "monkey_brain_core/build_condition.hpp"

namespace monkey_brain_core
{

std::vector<std::unique_ptr<Action>> ConreteOperationsFactory::create_actions(
  const ActionDescriptions & descriptions) const
{
  return build_actions(descriptions, env_, operator_registry_);
}

ConditionPtr ConreteOperationsFactory::create_condition(
  const ExpressionDescription & description) const
{
  return build_condition(description, env_, operator_registry_);
}

void ConreteOperationsFactory::register_operator(OperatorDefinition op)
{
  operator_registry_.register_operator(std::move(op.descriptor), std::move(op.make_fn));
}

} // namespace monkey_brain_core
