#ifndef MONKEY_BRAIN_SCXML_PARSE_SCXML_HPP
#define MONKEY_BRAIN_SCXML_PARSE_SCXML_HPP

#include <filesystem>

#include "monkey_brain_scxml/state_machine_description.hpp"

namespace monkey_brain_scxml
{

StateMachineDescription parse_scxml(const std::filesystem::path & path);

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_SCXML_PARSE_SCXML_HPP
