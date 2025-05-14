#ifndef MONKEY_BRAIN_STATE_MACHINE_OBSERVER_HPP
#define MONKEY_BRAIN_STATE_MACHINE_OBSERVER_HPP

#include "monkey_brain_core/context.hpp"

#include "monkey_brain_scxml/states.hpp"
#include "monkey_brain_scxml/state_name_range.hpp"

#include <memory>

namespace monkey_brain_scxml
{

class StateMachineObserver
{
public:
  virtual ~StateMachineObserver() = default;
  virtual void initialize(const std::vector<State> & states) = 0;
  virtual void on_state_change(StateNameRange currently_active_states) = 0;
};

class StateMachineObserverFactory
{
public:
  virtual ~StateMachineObserverFactory() = default;
  virtual std::unique_ptr<StateMachineObserver> create(monkey_brain_core::Context * context) const =
  0;
};

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_STATE_MACHINE_OBSERVER_HPP
