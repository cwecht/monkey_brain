#ifndef MONKEY_BRAIN_CORE_CONCRETE_OPERATIONS_FACTORY_HPP
#define MONKEY_BRAIN_CORE_CONCRETE_OPERATIONS_FACTORY_HPP

#include "monkey_brain_core/environment.hpp"
#include "monkey_brain_core/operations_factory.hpp"
#include "monkey_brain_core/operator_registry.hpp"

#include <optional>
#include <vector>

namespace monkey_brain_core
{

class ConreteOperationsFactory : public OperationsFactory
{
public:
  explicit ConreteOperationsFactory(Environment & env)
  : env_{env} {}

  Actions create_actions(
    const ActionDescriptions & descriptions) const override;

  ConditionPtr create_condition(
    const ExpressionDescription & description) const override;

  void register_operator(OperatorDefinition op) override;

private:
  Environment & env_;
  OperatorRegistry operator_registry_;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_CONCRETE_OPERATIONS_FACTORY_HPP
