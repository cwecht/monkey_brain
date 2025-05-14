#ifndef MONKEY_BRAIN_STATE_MACHINE_DESCRIPTION_HPP
#define MONKEY_BRAIN_STATE_MACHINE_DESCRIPTION_HPP

#include "monkey_brain_core/expression_description.hpp"
#include "monkey_brain_core/action_description.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace monkey_brain_scxml
{

struct TransitionDescription
{
  std::string event;
  std::optional<monkey_brain_core::ExpressionDescription> condition;
  monkey_brain_core::ActionDescriptions action;
  std::string target;
  std::string type = "external";
};

struct NestedStates;

enum class StateType : std::uint8_t
{
  REGULAR = 0u, FINAL, HISTORY_SHALLOW, HISTORY_DEEP
};

struct StateDescription
{
  std::string name;
  std::vector<TransitionDescription> transitions;
  monkey_brain_core::ActionDescriptions on_entry_action;
  monkey_brain_core::ActionDescriptions on_exit_action;
  std::unique_ptr<NestedStates> nested;

  inline bool is_final() const {return type == StateType::FINAL;}
  inline bool is_history() const
  {
    return type == StateType::HISTORY_SHALLOW || type == StateType::HISTORY_DEEP;
  }

  StateType type = StateType::REGULAR;
};

struct Initial
{
  std::string state;
  monkey_brain_core::ActionDescriptions action;
};

struct NestedStates
{
  NestedStates(std::vector<StateDescription> s, Initial i)
  : states{std::move(s)}, initial{std::move(i)}, are_parallel{false} {}
  NestedStates(std::vector<StateDescription> s)
  : states{std::move(s)}, are_parallel{true} {}
  std::vector<StateDescription> states;
  Initial initial;
  bool are_parallel;
};

struct StateMachineDescription
{
  std::vector<StateDescription> states;
  Initial initial;
};

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_STATE_MACHINE_DESCRIPTION_HPP
