#ifndef MONKEY_BRAIN_STATES_HPP
#define MONKEY_BRAIN_STATES_HPP

#include "monkey_brain_core/action.hpp"
#include "monkey_brain_core/condition.hpp"

#include <string>
#include <vector>

namespace monkey_brain_scxml
{

struct State;

struct Transition
{
  std::string event;
  monkey_brain_core::ConditionPtr condition;
  monkey_brain_core::Actions action;
  std::vector<State const *> target_state;
  bool is_internal = false;
  State const * source;
};

struct State
{
  enum class Type : std::uint8_t
  {
    REGULAR = 0u, FINAL, HISTORY_SHALLOW, HISTORY_DEEP
  };
  std::string name;
  std::vector<Transition> transitions;
  monkey_brain_core::Actions on_entry_action;
  monkey_brain_core::Actions on_exit_action;
  std::vector<State> substates;
  std::vector<State const *> automatic_sub_states;
  monkey_brain_core::Actions init_actions;
  State const * parent_state = nullptr;
  Type type = Type::REGULAR;
  std::string done_event;
  std::size_t document_order_id = 0;

  inline bool is_final() const {return type == Type::FINAL;}
  inline bool is_history() const
  {
    return type == Type::HISTORY_SHALLOW || type == Type::HISTORY_DEEP;
  }
  inline bool is_parallel() const {return automatic_sub_states.empty() && not substates.empty();}
  inline bool is_compound() const
  {
    return (not automatic_sub_states.empty()) && (not substates.empty());
  }
  inline bool is_atomic() const {return substates.empty();}
};

struct States
{
  std::vector<State> states;
  std::vector<State const *> initial_states;
  monkey_brain_core::Actions init_actions;
};

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_STATES_HPP
