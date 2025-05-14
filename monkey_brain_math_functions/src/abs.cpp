#include "monkey_brain_core/operator_factory.hpp"
#include "monkey_brain_core/dynamic_cast_move.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

#include <cassert>
#include <cmath>

namespace monkey_brain_math_functions
{

template<typename Type>
class AbsFunction : public monkey_brain_core::FunctionExpression<Type>
{
public:
  AbsFunction(monkey_brain_core::ExpressionPtr child)
  : child_{monkey_brain_core::dynamic_cast_move<monkey_brain_core::Expression<Type>>(child)}
  {}

  using ReturnType = typename monkey_brain_core::Expression<Type>::ReturnType;
  ReturnType get() const final
  {
    return std::abs(child_->get());
  }

private:
  std::unique_ptr<monkey_brain_core::Expression<Type>> child_;
};

template<typename Type>
monkey_brain_core::ExpressionWithType make_abs(monkey_brain_core::Expressions p_expressions)
{
  assert(p_expressions.size() == 1);
  return {std::make_unique<AbsFunction<Type>>(std::move(p_expressions.front())),
    monkey_brain_core::name<Type>()};
}

class AbsOperatorFactory : public monkey_brain_core::OperatorFactory
{
public:
  monkey_brain_core::OperatorDefinitions create_operation() override
  {
    return {
      {{"abs", {monkey_brain_core::name<int64_t>()}, false}, &make_abs<int64_t>},
      {{"abs", {monkey_brain_core::name<double>()}, false}, &make_abs<double>}
    };
  }
};

} // namespace monkey_brain_math_functions

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_math_functions::AbsOperatorFactory,
  monkey_brain_core::OperatorFactory)
