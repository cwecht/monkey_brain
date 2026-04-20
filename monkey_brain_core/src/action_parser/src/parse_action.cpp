#include "monkey_brain_core/parse_action.hpp"

#include "monkey_brain_core/parse_expression.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <set>

namespace monkey_brain_core
{

namespace
{

using Lines = std::vector<std::string>;

Lines tokenize_lines(const std::string & action)
{
  Lines lines;
  boost::split(
    lines, action, boost::is_any_of(";\n"),
    boost::token_compress_on);
  return lines;
}

void trim_all(Lines & lines)
{
  for (std::string & l : lines) {
    boost::trim(l);
  }
}

ActionDescription parse_line(std::string action)
{
  if (const auto pos = action.find(":="); pos != std::string::npos) {
    std::string first{action.substr(0, pos)};
    boost::trim(first);
    auto first_expression = parse_expression(first);
    std::string second{action.substr(pos + 2)};
    boost::trim(second);
    auto second_expression = parse_expression(second);
    if (first_expression and second_expression) {
      return {":=", {std::move(*first_expression), std::move(*second_expression)}};
    } else {
      throw std::invalid_argument("invalid assignment action: " + action);
    }
  }

  const auto first_space = action.find(' ');
  const auto command = action.substr(0, first_space);
  if (command == "PERFORM" || command == "RAISE") {
    const std::string argument{action.substr(first_space + 1)};
    auto argument_expression = parse_expression(argument);
    if (argument_expression) {
      return {command, {std::move(*argument_expression)}};
    }
  }

  throw std::invalid_argument("invalid action: " + action);
}

ActionDescriptions parse_action(Lines lines)
{
  trim_all(lines);
  lines.erase(std::remove(lines.begin(), lines.end(), ""), lines.end());
  ActionDescriptions actions;
  actions.reserve(lines.size());
  std::transform(
    lines.begin(), lines.end(), std::back_inserter(actions),
    &parse_line);
  return actions;
}
}  // namespace

ActionDescriptions parse_action(const std::string & action)
{
  if (action.empty()) {
    return {};
  } else {
    return parse_action(tokenize_lines(action));
  }
}

} // namespace monkey_brain_core
