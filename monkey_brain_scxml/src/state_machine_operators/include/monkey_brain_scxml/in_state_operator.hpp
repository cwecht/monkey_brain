#ifndef MONKEY_BRAIN_SCXML_IN_STATE_CONDITION_HPP
#define MONKEY_BRAIN_SCXML_IN_STATE_CONDITION_HPP

#include "monkey_brain_scxml/state_machine.hpp"
#include "monkey_brain_core/expression.hpp"
#include "monkey_brain_core/operations_factory.hpp"

namespace monkey_brain_scxml
{

monkey_brain_core::OperatorDefinition create_in_state_operator(StateMachine * sm);

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_SCXML_IN_STATE_CONDITION_HPP
