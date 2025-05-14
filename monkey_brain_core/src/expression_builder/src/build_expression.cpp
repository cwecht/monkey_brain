#include "monkey_brain_core/build_expression.hpp"

#include "monkey_brain_core/condition.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace monkey_brain_core
{
namespace
{

template<typename ValueType, typename UnderlyingType = ValueType>
class EnvironemntValue : public Expression<ValueType>
{
public:
  explicit EnvironemntValue(const Environment & env, std::string_view ref)
  : ptr_{env.get_value<UnderlyingType>(ref)} {}

  using ReturnType = typename Expression<ValueType>::ReturnType;

  virtual ReturnType get() const final
  {
    return *ptr_;
  }

private:
  const UnderlyingType * const ptr_;
};

template<>
class EnvironemntValue<void>: public Expression<void>
{
public:
  explicit EnvironemntValue(const Environment & env, std::string_view ref)
  : ptr_{env.get_value<void>(ref)} {}

  virtual void const * get() const final
  {
    return ptr_;
  }

private:
  const void * const ptr_;
};

template<typename ValueType>
class ConstantValue : public Expression<ValueType>
{
public:
  explicit ConstantValue(const ValueType & value)
  : value_{value} {}

  using ReturnType = typename Expression<ValueType>::ReturnType;

  virtual ReturnType get() const final
  {
    return value_;
  }

private:
  ValueType value_;
};

template<typename ValueType>
using BinaryOperation = ValueType (*)(ExpressionReturnType<ValueType>,
      ExpressionReturnType<ValueType>);

template<typename ValueType, typename UnderlyingType = ValueType>
ExpressionWithType make_value(
  const std::string & argument,
  const Environment & env, std::optional<std::string> type_name = {})
{
  return {std::make_unique<EnvironemntValue<ValueType, UnderlyingType>>(env, argument),
    type_name.value_or(name<ValueType>())};
}

struct ConstantValueMaker
{
  ExpressionWithType operator()(std::monostate) {return {};}
  template<typename ValueType>
  ExpressionWithType operator()(ValueType val)
  {
    return {std::make_unique<ConstantValue<ValueType>>(std::move(val)), name<ValueType>()};
  }
};

ExpressionWithType build_value(const ExpressionDescription & val)
{
  return std::visit(ConstantValueMaker{}, val.arg);
}

ExpressionWithType build_reference(
  const ExpressionDescription & val,
  const Environment & env)
{
  ValueType type = env.get_type_of(std::get<std::string>(val.arg)).value_or(type);
  using MakerFn = ExpressionWithType(*)(const std::string &, const Environment &,
        std::optional<std::string>);
  static const std::map<ValueType, MakerFn> kTypesToMakerFn = {
    {ValueTypes::BOOL, (MakerFn) & make_value<bool, bool>},
    {ValueTypes::CHAR, (MakerFn) & make_value<char, char>},
    {ValueTypes::FLOAT32, (MakerFn) & make_value<double, float>},
    {ValueTypes::FLOAT64, (MakerFn) & make_value<double, double>},
    {ValueTypes::INT8, (MakerFn) & make_value<int64_t, int8_t>},
    {ValueTypes::UINT8, (MakerFn) & make_value<uint64_t, uint8_t>},
    {ValueTypes::INT16, (MakerFn) & make_value<int64_t, int16_t>},
    {ValueTypes::UINT16, (MakerFn) & make_value<uint64_t, uint16_t>},
    {ValueTypes::INT32, (MakerFn) & make_value<int64_t, int32_t>},
    {ValueTypes::UINT32, (MakerFn) & make_value<uint64_t, uint32_t>},
    {ValueTypes::INT64, (MakerFn) & make_value<int64_t, int64_t>},
    {ValueTypes::UINT64, (MakerFn) & make_value<uint64_t, uint64_t>},
    {ValueTypes::STRING, (MakerFn) & make_value<std::string, std::string>}};
  auto m = kTypesToMakerFn.find(type);
  if (m == kTypesToMakerFn.end()) {
    return make_value<void>(std::get<std::string>(val.arg), env, type);
  }
  return (*m->second)(std::get<std::string>(val.arg), env, {});
}

struct ExpressionsWithTypes
{
  Expressions expressions;
  std::vector<ValueType> types;
};

ExpressionsWithTypes build_expressions(
  const std::vector<ExpressionDescription> & sub_conditions,
  const Environment & env,
  const OperatorRegistry & operator_registry)
{
  Expressions vs;
  vs.reserve(sub_conditions.size());
  std::vector<ValueType> types;
  types.reserve(sub_conditions.size());
  for (const auto & sc : sub_conditions) {
    auto et = build_expression(sc, env, operator_registry);
    vs.push_back(std::move(et.expression));
    types.push_back(std::move(et.type));
  }
  return {std::move(vs), std::move(types)};
}

}  // namespace

ExpressionWithType build_expression(
  const ExpressionDescription & description,
  const Environment & env,
  const OperatorRegistry & operator_registry)
{
  if (description.type == "VALUE") {
    return build_value(description);
  }

  if (description.type == "REFERENCE") {
    return build_reference(description, env);
  }

  auto vs = build_expressions(description.sub_expressions, env, operator_registry);
  return operator_registry.create_expression(
    {description.type, std::move(vs.types)}, std::move(vs.expressions));
}

} // namespace monkey_brain_core
