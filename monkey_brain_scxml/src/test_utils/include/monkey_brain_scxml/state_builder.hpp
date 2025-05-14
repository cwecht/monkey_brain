#ifndef MONKEY_BRAIN_SCXML_STATE_BUILDER_HPP
#define MONKEY_BRAIN_SCXML_STATE_BUILDER_HPP

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_core/action.hpp"

#include "monkey_brain_scxml/states.hpp"

#include <memory>
#include <vector>

namespace monkey_brain_scxml
{

struct ActionMock : monkey_brain_core::Action
{
  MOCK_METHOD0(execute, void());
};

monkey_brain_core::Actions as_actions(std::unique_ptr<ActionMock> x)
{
  monkey_brain_core::Actions v;
  if (x != nullptr) {
    v.push_back(std::move(x));
  }
  return v;
}

class StateBuilder
{
public:
  explicit StateBuilder(std::string name)
  : state_{std::move(name), {}, {}, {}, {}, {}, {}, nullptr, State::Type::REGULAR, "",
      document_order_id++} {}

  StateBuilder with_on_entry_action(std::unique_ptr<ActionMock> a) &&
  {
    state_.on_entry_action = as_actions(std::move(a));
    return std::move(*this);
  }

  StateBuilder with_on_exit_action(std::unique_ptr<ActionMock> a) &&
  {
    state_.on_exit_action = as_actions(std::move(a));
    return std::move(*this);
  }

  StateBuilder with_parent(State const * parent) &&
  {
    state_.parent_state = parent;
    return std::move(*this);
  }


  State build() &&
  {
    return std::move(state_);
  }

private:
  static inline std::size_t document_order_id = 0;
  State state_;
};

void add_transition(
  State & state, std::string event, std::vector<State const *> targets,
  std::unique_ptr<ActionMock> a = nullptr)
{
  state.transitions.push_back(
    Transition{std::move(event), nullptr, as_actions(
        std::move(
          a)), std::move(targets), false, &state});
}

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_SCXML_STATE_BUILDER_HPP
