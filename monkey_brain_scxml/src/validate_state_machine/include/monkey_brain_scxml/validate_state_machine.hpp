#ifndef MONKEY_BRAIN_SCXML_VALIDATE_STATE_MACHINE__HPP
#define MONKEY_BRAIN_SCXML_VALIDATE_STATE_MACHINE__HPP

#include "monkey_brain_core/environment.hpp"
#include "monkey_brain_scxml/state_machine_description.hpp"

namespace monkey_brain_scxml
{

struct ValidationError
{
  std::string message;
};

using ValidationErrors = std::vector<ValidationError>;

ValidationErrors validate_state_machine(
  const StateMachineDescription & sds,
  const monkey_brain_core::Environment & env);

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_SCXML_VALIDATE_STATE_MACHINE__HPP
