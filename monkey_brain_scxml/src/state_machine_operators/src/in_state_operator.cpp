#include "monkey_brain_scxml/in_state_operator.hpp"

#include "monkey_brain_core/dynamic_cast_move.hpp"
#include "monkey_brain_core/expression.hpp"

template<typename Range, typename T>
bool contains(const Range & rng, const T & value)
{
  return std::end(rng) != std::find(std::begin(rng), std::end(rng), value);
}

namespace monkey_brain_scxml
{

class InStateCondition : public monkey_brain_core::FunctionExpression<bool>
{
public:
  InStateCondition(StateMachine * sm, monkey_brain_core::ExpressionPtr state_name)
  : sm_{sm}
    , state_name_{monkey_brain_core::dynamic_cast_move<monkey_brain_core::Expression<std::string>>(
        state_name)}
  {}

  bool get() const final
  {
    return contains(sm_->active_states(), state_name_->get());
  }

private:
  StateMachine * sm_;
  std::unique_ptr<monkey_brain_core::Expression<std::string>> state_name_;
};

monkey_brain_core::ExpressionWithType make_in_state(
  StateMachine * sm,
  monkey_brain_core::Expressions p_expressions)
{
  return {std::make_unique<InStateCondition>(sm, std::move(p_expressions.front())),
    monkey_brain_core::ValueTypes::BOOL};
}

monkey_brain_core::OperatorDefinition create_in_state_operator(StateMachine * sm)
{
  monkey_brain_core::ExpressionMakerFn make_in_state_fn =
    [sm] (monkey_brain_core::Expressions p_expressions) {
      return make_in_state(sm, std::move(p_expressions));
    };
  return {{"in_state", {monkey_brain_core::ValueTypes::STRING}}, std::move(make_in_state_fn)};
}

} // namespace monkey_brain_scxml
