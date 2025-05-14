#ifndef MONKEY_BRAIN_CORE_BUILD_EXPRESSION_HPP
#define MONKEY_BRAIN_CORE_BUILD_EXPRESSION_HPP

#include "monkey_brain_core/expression_description.hpp"
#include "monkey_brain_core/environment.hpp"
#include "monkey_brain_core/expression.hpp"

#include "monkey_brain_core/operator_registry.hpp"

namespace monkey_brain_core
{

ExpressionWithType build_expression(
  const ExpressionDescription & description,
  const Environment & env,
  const OperatorRegistry & operator_registry = {});

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_BUILD_EXPRESSION_HPP
