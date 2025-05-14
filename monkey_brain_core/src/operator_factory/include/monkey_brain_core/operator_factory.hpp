#ifndef MONKEY_BRAIN_CORE_OPERATOR_FACTORY_HPP
#define MONKEY_BRAIN_CORE_OPERATOR_FACTORY_HPP

#include "monkey_brain_core/expression.hpp"
#include "monkey_brain_core/value_types.hpp"

#include <functional>
#include <vector>

namespace monkey_brain_core
{

struct OperatorDescriptor
{
  std::string_view name;
  std::vector<ValueType> parameter_types;
  bool last_is_variadic = false;
};

using ExpressionMakerFn = std::function<ExpressionWithType(Expressions)>;

struct OperatorDefinition
{
  OperatorDescriptor descriptor;
  ExpressionMakerFn make_fn;
};

using OperatorDefinitions = std::vector<OperatorDefinition>;

class OperatorFactory
{
public:
  virtual ~OperatorFactory() = default;
  virtual OperatorDefinitions create_operation() = 0;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_OPERATOR_FACTORY_HPP
