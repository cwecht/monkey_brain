#include "monkey_brain_core/operator_registry.hpp"

#include "monkey_brain_core/condition.hpp"
#include "monkey_brain_core/dynamic_cast_move.hpp"

#include <boost/functional/hash.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace monkey_brain_core
{
namespace
{

template<typename ValueType>
using BinaryOperation = ValueType (*)(ExpressionReturnType<ValueType>,
      ExpressionReturnType<ValueType>);

template<typename ValueType>
class BinaryVariadicExpression : public FunctionExpression<ValueType>
{
  using LocalExpressionPtr = std::unique_ptr<Expression<ValueType>>;
  using LocalExpressions = std::vector<LocalExpressionPtr>;

public:
  using ReturnType = typename Expression<ValueType>::ReturnType;
  BinaryVariadicExpression(BinaryOperation<ValueType> fun, Expressions p_expressions)
  : fun_{fun}
    , expressions_{convert(std::move(p_expressions))} {}

  virtual ReturnType get() const final
  {
    const auto op = [this](ExpressionReturnType<ValueType> akk, const LocalExpressionPtr & v)
      {return fun_(akk, v->get());};
    assert(not expressions_.empty());
    return this->store(
      std::accumulate(
        std::begin(expressions_) + 1,
        std::end(expressions_), expressions_.front()->get(), op));
  }

private:
  static LocalExpressions convert(Expressions p_expressions)
  {
    LocalExpressions vs;
    vs.reserve(p_expressions.size());
    for (ExpressionPtr & v : p_expressions) {
      vs.emplace_back(dynamic_cast_move<Expression<ValueType>>(v));
    }
    return vs;
  }
  BinaryOperation<ValueType> fun_;
  LocalExpressions expressions_;
};

class NotExpression : public Expression<bool>
{
public:
  explicit NotExpression(ExpressionPtr p_sub_condition)
  : sub_condition_{dynamic_cast_move<Expression<bool>>(p_sub_condition)} {}

  bool get() const final
  {
    return not sub_condition_->get();
  }

private:
  ConditionPtr sub_condition_;
};

using LogicalOperator = bool (*)(bool, bool);

ExpressionPtr make_logical_expression(LogicalOperator op, Expressions p_expressions)
{
  return std::make_unique<BinaryVariadicExpression<bool>>(op, std::move(p_expressions));
}

template<typename ValueType, BinaryOperation<ValueType> fun>
ExpressionWithType make_function(Expressions p_expressions)
{
  return {std::make_unique<BinaryVariadicExpression<ValueType>>(
    fun, std::move(p_expressions)), name<ValueType>()};
}

template<typename V>
V plus(ExpressionReturnType<V> lhs, ExpressionReturnType<V> rhs)
{
  return lhs + rhs;
}

template<typename V>
V minus(ExpressionReturnType<V> lhs, ExpressionReturnType<V> rhs)
{
  return lhs - rhs;
}

template<typename V>
V modulus(ExpressionReturnType<V> lhs, ExpressionReturnType<V> rhs)
{
  if constexpr (std::is_floating_point_v<V>) {
    return std::fmod(lhs, rhs);
  } else {
    return lhs % rhs;
  }
}

template<typename V>
V multiplies(ExpressionReturnType<V> lhs, ExpressionReturnType<V> rhs)
{
  return lhs * rhs;
}

template<typename V>
V divides(ExpressionReturnType<V> lhs, ExpressionReturnType<V> rhs)
{
  return lhs / rhs;
}

template<typename Value>
void register_arithmetic_operators(OperatorRegistry & registry)
{
  const ValueType value_type = name<Value>();
  registry.register_operator({"+", {value_type}, true}, &make_function<Value, &plus<Value>>);

  if constexpr (not std::is_same_v<Value, std::string>) {
    registry.register_operator({"-", {value_type}, true}, &make_function<Value, &minus<Value>>);
    registry.register_operator({"%", {value_type}, true}, &make_function<Value, &modulus<Value>>);
    registry.register_operator({"/", {value_type}, true}, &make_function<Value, &divides<Value>>);
    registry.register_operator({"*", {value_type}, true},
          &make_function<Value, &multiplies<Value>>);
  }
}

template<typename ValueType>
using Comparison = bool (*)(ExpressionReturnType<ValueType>, ExpressionReturnType<ValueType>);

template<typename ValueType>
class ComparisonCondition : public Expression<bool>
{
  using LocalExpression = Expression<ValueType>;
  using LocalExpressionPtr = std::unique_ptr<LocalExpression>;

public:
  using Comp = Comparison<ValueType>;
  ComparisonCondition(
    Comp p_comperator,
    ExpressionPtr p_first_value,
    ExpressionPtr p_second_value)
  : comperator_{p_comperator}
    , first_value_{dynamic_cast_move<LocalExpression>(p_first_value)}
    , second_value_{dynamic_cast_move<LocalExpression>(p_second_value)}
  {
    assert(first_value_ != nullptr);
    assert(second_value_ != nullptr);
  }

  bool get() const final
  {
    return comperator_(first_value_->get(), second_value_->get());
  }

private:
  Comp comperator_;
  LocalExpressionPtr first_value_;
  LocalExpressionPtr second_value_;
};

template<typename ValueType, Comparison<ValueType> comperator>
ExpressionWithType make_comparison_condition(Expressions expressions)
{
  return {std::make_unique<ComparisonCondition<ValueType>>(
    comperator, std::move(expressions.front()), std::move(expressions.back())), ValueTypes::BOOL};
}

template<typename ValueType>
bool equals(ExpressionReturnType<ValueType> lhs, ExpressionReturnType<ValueType> rhs)
{
  return lhs == rhs;
}

template<typename ValueType>
bool not_equals(ExpressionReturnType<ValueType> lhs, ExpressionReturnType<ValueType> rhs)
{
  return lhs != rhs;
}

template<typename ValueType>
bool less_than(ExpressionReturnType<ValueType> lhs, ExpressionReturnType<ValueType> rhs)
{
  return lhs < rhs;
}

template<typename ValueType>
bool greater_than(ExpressionReturnType<ValueType> lhs, ExpressionReturnType<ValueType> rhs)
{
  return lhs > rhs;
}

template<typename ValueType>
bool less_equals(ExpressionReturnType<ValueType> lhs, ExpressionReturnType<ValueType> rhs)
{
  return lhs <= rhs;
}

template<typename ValueType>
bool greater_equals(ExpressionReturnType<ValueType> lhs, ExpressionReturnType<ValueType> rhs)
{
  return lhs >= rhs;
}

template<typename Value>
void register_comparison_operators(OperatorRegistry & registry)
{
  const ValueType value_type = name<Value>();
  registry.register_operator({"==", {value_type, value_type}, false},
      &make_comparison_condition<Value, &equals<Value>>);
  registry.register_operator({"!=", {value_type, value_type}, false},
      &make_comparison_condition<Value, &not_equals<Value>>);
  registry.register_operator({"<", {value_type, value_type}, false},
      &make_comparison_condition<Value, &less_than<Value>>);
  registry.register_operator({">", {value_type, value_type}, false},
      &make_comparison_condition<Value, &greater_than<Value>>);
  registry.register_operator({"<=", {value_type, value_type}, false},
      &make_comparison_condition<Value, &less_equals<Value>>);
  registry.register_operator({">=", {value_type, value_type}, false},
      &make_comparison_condition<Value, &greater_equals<Value>>);
}

ExpressionWithType make_NOT(Expressions expressions)
{
  return {std::make_unique<NotExpression>(std::move(expressions.front())), ValueTypes::BOOL};
}

ExpressionWithType make_AND(Expressions expressions)
{
  return {make_logical_expression(
      [](bool lhs, bool rhs) {return lhs and rhs;}, std::move(expressions)), ValueTypes::BOOL};
}

ExpressionWithType make_OR(Expressions expressions)
{
  return {make_logical_expression(
      [](bool lhs, bool rhs) {return lhs or rhs;}, std::move(expressions)), ValueTypes::BOOL};
}

template<typename TargetType, typename SourceType>
class CastExpression : public FunctionExpression<TargetType>
{
  using LocalExpressionPtr = std::unique_ptr<Expression<SourceType>>;

public:
  using ReturnType = typename Expression<TargetType>::ReturnType;
  CastExpression(ExpressionPtr p_expression)
  : expression_{dynamic_cast_move<Expression<SourceType>>(std::move(p_expression))}
  {
    assert(expression_ != nullptr);
  }

  virtual ReturnType get() const final
  {
    return static_cast<TargetType>(expression_->get());
  }

private:
  LocalExpressionPtr expression_;
};

template<typename TargetType, typename SourceType>
ExpressionWithType make_cast(Expressions source)
{
  assert(source.size() == 1);
  return {std::make_unique<CastExpression<TargetType, SourceType>>(std::move(source.front())),
    name<TargetType>()};
}

template<typename TargetType, typename SourceType>
void register_cast(OperatorRegistry & registry, std::string_view cast_name)
{
  registry.register_operator({cast_name, {name<SourceType>()}, false},
        &make_cast<TargetType, SourceType>);
}

template<typename SourceType>
void register_casts_from(OperatorRegistry & registry)
{
  register_cast<uint64_t, SourceType>(registry, "UNSIGNED");
  register_cast<int64_t, SourceType>(registry, "SIGNED");
  register_cast<double, SourceType>(registry, "FLOAT");
}

}  // namespace

OperatorRegistry::OperatorRegistry()
{
  register_arithmetic_operators<double>(*this);
  register_arithmetic_operators<int64_t>(*this);
  register_arithmetic_operators<uint64_t>(*this);
  register_arithmetic_operators<std::string>(*this);

  register_comparison_operators<double>(*this);
  register_comparison_operators<int64_t>(*this);
  register_comparison_operators<uint64_t>(*this);
  register_comparison_operators<std::string>(*this);

  register_operator({"NOT", {ValueTypes::BOOL}, false}, &make_NOT);
  register_operator({"AND", {ValueTypes::BOOL}, true}, &make_AND);
  register_operator({"OR", {ValueTypes::BOOL}, true}, &make_OR);

  register_casts_from<int64_t>(*this);
  register_casts_from<uint64_t>(*this);
  register_casts_from<double>(*this);
}

ExpressionWithType OperatorRegistry::create_expression(
  OperatorDescriptor op_desc,
  Expressions p_expressions) const
{
  auto m = to_functional_expression_maker.find(op_desc);
  if (m == to_functional_expression_maker.end()) {
    const auto ps = op_desc.parameter_types;
    std::stringstream os;
    std::copy(ps.begin(), ps.end(), std::ostream_iterator<ValueType>(os, ", "));
    throw std::runtime_error("No operator named \"" + std::string{op_desc.name} +
        "\" with input types: (" + os.str() + ")");
  }
  return m->second(std::move(p_expressions));
}

void OperatorRegistry::register_operator(OperatorDescriptor descriptor, ExpressionMakerFn maker_fn)
{
  to_functional_expression_maker.emplace(std::move(descriptor), std::move(maker_fn));
}

std::size_t OperatorRegistry::Hash::operator()(const OperatorDescriptor & s) const noexcept
{
  const auto & p_types = s.parameter_types;
  auto end = std::adjacent_find(p_types.rbegin(), p_types.rend(), std::not_equal_to{}).base();
  std::size_t h = boost::hash_range(p_types.begin(), end);
  boost::hash_combine(h, s.name);
  return h;
}

bool OperatorRegistry::EqualTo::operator()(
  const OperatorDescriptor & a,
  const OperatorDescriptor & b) const
{
  if (a.name != b.name) {
    return false;
  }
  if (a.last_is_variadic == b.last_is_variadic) {
    return a.parameter_types == b.parameter_types;
  }
  const auto & variadic = a.last_is_variadic ? a.parameter_types : b.parameter_types;
  const auto & non_var = not a.last_is_variadic ? a.parameter_types : b.parameter_types;

  if (variadic.size() > non_var.size()) {
    return false;
  }
  auto params = variadic;
  params.resize(non_var.size(), params.back());
  return params == non_var;
}

} // namespace monkey_brain_core
