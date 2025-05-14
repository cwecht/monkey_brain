#ifndef MONKEY_BRAIN_SCXML_BUILD_STATES_HPP
#define MONKEY_BRAIN_SCXML_BUILD_STATES_HPP

#include "monkey_brain_core/operations_factory.hpp"
#include "monkey_brain_scxml/states.hpp"
#include "monkey_brain_scxml/state_machine_description.hpp"

namespace monkey_brain_scxml
{

States build_states(
  const StateMachineDescription & state_machine_description,
  const monkey_brain_core::OperationsFactory & ops_factory);

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_SCXML_BUILD_STATES_HPP
