#ifndef MONKEY_BRAIN_CORE_PARSE_ACTION_HPP
#define MONKEY_BRAIN_CORE_PARSE_ACTION_HPP

#include "monkey_brain_core/action_description.hpp"

#include <string>

namespace monkey_brain_core
{

ActionDescriptions parse_action(const std::string & action);

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_PARSE_ACTION_HPP
