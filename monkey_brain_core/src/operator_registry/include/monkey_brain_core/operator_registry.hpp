#ifndef MONKEY_BRAIN_CORE_OPERATOR_REGISTRY_HPP
#define MONKEY_BRAIN_CORE_OPERATOR_REGISTRY_HPP

#include "monkey_brain_core/environment.hpp"
#include "monkey_brain_core/expression.hpp"
#include "monkey_brain_core/operator_factory.hpp"
#include "monkey_brain_core/value_types.hpp"

#include <functional>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace monkey_brain_core
{

class OperatorRegistry
{
public:
  OperatorRegistry();
  ExpressionWithType create_expression(
    OperatorDescriptor op_desc,
    Expressions p_expressions) const;

  void register_operator(OperatorDescriptor descriptor, ExpressionMakerFn maker_fn);

private:
  struct Hash
  {
    std::size_t operator()(const OperatorDescriptor & s) const noexcept;
  };

  struct EqualTo
  {
    bool operator()(const OperatorDescriptor & a, const OperatorDescriptor & b) const;
  };

  using TypesAndOpsToExpressionMakerFn =
    std::unordered_map<OperatorDescriptor, ExpressionMakerFn, Hash, EqualTo>;

  TypesAndOpsToExpressionMakerFn to_functional_expression_maker;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_OPERATOR_REGISTRY_HPP
