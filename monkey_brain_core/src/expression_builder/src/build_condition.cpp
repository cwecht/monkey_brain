#include "monkey_brain_core/build_condition.hpp"

#include "monkey_brain_core/build_expression.hpp"
#include "monkey_brain_core/dynamic_cast_move.hpp"

#include <stdexcept>

namespace monkey_brain_core
{

ConditionPtr build_condition(
  const ExpressionDescription & description,
  const Environment & env,
  const OperatorRegistry & operator_registry)
{
  auto exp = build_expression(description, env, operator_registry);
  if (ValueTypes::BOOL != exp.type) {
    throw std::runtime_error("expression is not a condition");
  }
  return dynamic_cast_move<Condition>(std::move(exp.expression));
}

} // namespace monkey_brain_core
