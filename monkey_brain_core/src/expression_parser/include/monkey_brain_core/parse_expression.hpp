#ifndef MONKEY_BRAIN_CORE_PARSE_EXPRESSION_HPP
#define MONKEY_BRAIN_CORE_PARSE_EXPRESSION_HPP

#include "monkey_brain_core/expression_description.hpp"

#include <optional>

namespace monkey_brain_core
{

std::optional<ExpressionDescription> parse_expression(const std::string & expression);

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_PARSE_EXPRESSION_HPP
