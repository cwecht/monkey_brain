#ifndef MONKEY_BRAIN_CORE_BUILD_ACTIONS_HPP
#define MONKEY_BRAIN_CORE_BUILD_ACTIONS_HPP

#include "monkey_brain_core/action.hpp"
#include "monkey_brain_core/action_description.hpp"
#include "monkey_brain_core/environment.hpp"

#include "monkey_brain_core/operator_registry.hpp"

#include <memory>
#include <vector>

namespace monkey_brain_core
{

std::vector<std::unique_ptr<Action>> build_actions(
  const ActionDescriptions & descriptions,
  Environment & env,
  const OperatorRegistry & operator_registry = {});

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_BUILD_ACTIONS_HPP
