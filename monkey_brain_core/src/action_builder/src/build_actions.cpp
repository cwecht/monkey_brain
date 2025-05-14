#include "monkey_brain_core/build_actions.hpp"

#include "monkey_brain_core/build_expression.hpp"
#include "monkey_brain_core/dynamic_cast_move.hpp"

#include <boost/lexical_cast.hpp>

#include <stdexcept>

namespace monkey_brain_core
{

namespace
{
class PerformAction : public Action
{
public:
  explicit PerformAction(std::string ref, Environment & env)
  : reference_{std::move(ref)}, env_{env} {}

  void execute() final
  {
    env_.perform(reference_);
  }

private:
  std::string reference_;
  Environment & env_;
};

template<typename ValueT, bool is_cheap_to_copy = is_check_to_return_by_value<ValueT>>
class AssignAction;

template<typename T>
struct InternalRepresentation { using type = T; };

template<>
struct InternalRepresentation<int8_t> { using type = int64_t; };

template<>
struct InternalRepresentation<int16_t> { using type = int64_t; };

template<>
struct InternalRepresentation<int32_t> { using type = int64_t; };

template<>
struct InternalRepresentation<uint8_t> { using type = uint64_t; };

template<>
struct InternalRepresentation<uint16_t> { using type = uint64_t; };

template<>
struct InternalRepresentation<uint32_t> { using type = uint64_t; };

template<>
struct InternalRepresentation<float> { using type = double; };

bool is_assignable_to(ValueType source, ValueType target)
{
  return (source == target) ||
         (is_signed_integer(source) and is_signed_integer(target)) ||
         (is_unsigned_integer(source) and is_unsigned_integer(target)) ||
         (is_floating_point(source) and is_floating_point(target));
}

template<typename ValueT>
class AssignAction<ValueT, true>: public Action
{
public:
  using InternalT = typename InternalRepresentation<ValueT>::type;
  explicit AssignAction(std::string target, ExpressionPtr source, Environment & env)
  : target_{std::move(target)}
    , source_{dynamic_cast_move<Expression<InternalT>>(source)}
    , env_{env}
  {
    assert(source_ != nullptr);
  }

  void execute() final
  {
    const ValueT val = static_cast<ValueT>(source_->get());
    env_.assign_value(target_, &val);
  }

private:
  std::string target_;
  std::unique_ptr<Expression<InternalT>> source_;
  Environment & env_;
};

template<typename ValueT>
class AssignAction<ValueT, false>: public Action
{
public:
  explicit AssignAction(std::string target, ExpressionPtr source, Environment & env)
  : target_{std::move(target)}
    , source_{dynamic_cast_move<Expression<ValueT>>(source)}
    , env_{env} {}

  void execute() final
  {
    env_.assign_value(target_, static_cast<const void *>(&source_->get()));
  }

private:
  std::string target_;
  std::unique_ptr<Expression<ValueT>> source_;
  Environment & env_;
};

template<>
class AssignAction<void, false>: public Action
{
public:
  explicit AssignAction(std::string target, ExpressionPtr source, Environment & env)
  : target_{std::move(target)}
    , source_(dynamic_cast_move<Expression<void>>(source))
    , env_{env} {}

  void execute() final
  {
    env_.assign_value(target_, source_->get());
  }

private:
  std::string target_;
  std::unique_ptr<Expression<void>> source_;
  Environment & env_;
};

template<typename ValueType>
std::unique_ptr<Action> make_assign_action(
  std::string target, ExpressionPtr expr,
  Environment & env)
{
  return std::make_unique<AssignAction<ValueType>>(std::move(target), std::move(expr), env);
}

std::unique_ptr<Action>
build_assign_action(ValueType type, std::string target, ExpressionPtr expr, Environment & env)
{
  using MakerFn = std::unique_ptr<Action>(*)(std::string, ExpressionPtr, Environment &);
  static const std::map<ValueType, MakerFn> kTypesToMakerFn {
    {ValueTypes::BOOL, &make_assign_action<bool>},
    {ValueTypes::CHAR, &make_assign_action<char>},
    {ValueTypes::FLOAT32, &make_assign_action<float>},
    {ValueTypes::FLOAT64, &make_assign_action<double>},
    {ValueTypes::INT8, &make_assign_action<int8_t>},
    {ValueTypes::UINT8, &make_assign_action<uint8_t>},
    {ValueTypes::INT16, &make_assign_action<int16_t>},
    {ValueTypes::UINT16, &make_assign_action<uint16_t>},
    {ValueTypes::INT32, &make_assign_action<int32_t>},
    {ValueTypes::UINT32, &make_assign_action<uint32_t>},
    {ValueTypes::INT64, &make_assign_action<int64_t>},
    {ValueTypes::UINT64, &make_assign_action<uint64_t>},
    {ValueTypes::STRING, &make_assign_action<std::string>}
  };

  auto m = kTypesToMakerFn.find(type);
  if (m == kTypesToMakerFn.end()) {
    return make_assign_action<void>(target, std::move(expr), env);
  }
  return (*m->second)(target, std::move(expr), env);
}

std::unique_ptr<Action> build_action(
  const ActionDescription & description,
  Environment & env,
  const OperatorRegistry & operator_registry)
{
  auto const & args = description.arguments;
  // TODO: for PERFORM: check whether target exists
  if ((description.type == "PERFORM" || description.type == "RAISE") && args.size() == 1) {
    return std::make_unique<PerformAction>(std::get<std::string>(args.front().arg), env);
  }
  if (description.type == ":=" && args.size() == 2) {
    const auto & target_desc = args.front();
    if (target_desc.type != "REFERENCE") {
      throw std::runtime_error("The target of an assignment must be a reference and not a " +
            target_desc.type);
    }
    const auto & target_reference = std::get<std::string>(args.front().arg);
    const auto target_type = env.get_type_of(target_reference);
    if (not target_type) {
      throw std::runtime_error(target_reference + " could not be resolved!");
    }
    auto source_expr = build_expression(args.back(), env, operator_registry);
    if (not is_assignable_to(source_expr.type, *target_type)) {
      throw std::runtime_error("Can not assign: " + source_expr.type + " is not assignable to " +
            *target_type);
    }

    return build_assign_action(*target_type, target_reference, std::move(source_expr.expression),
          env);
  }

  throw std::invalid_argument("invalid action");
}

} // namespace

std::vector<std::unique_ptr<Action>> build_actions(
  const ActionDescriptions & descriptions,
  Environment & env,
  const OperatorRegistry & operator_registry)
{
  std::vector<std::unique_ptr<Action>> actions;
  actions.reserve(descriptions.size());
  for (const auto & description : descriptions) {
    actions.emplace_back(build_action(description, env, operator_registry));
  }

  return actions;
}

} // namespace monkey_brain_core
