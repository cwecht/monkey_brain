#ifndef MONKEY_BRAIN_CORE_ACTION_DESCRIPTION_HPP
#define MONKEY_BRAIN_CORE_ACTION_DESCRIPTION_HPP

#include "monkey_brain_core/expression_description.hpp"

#include <string>
#include <vector>

#include <unordered_set>

namespace monkey_brain_core
{

struct ActionDescription
{
  std::string type;
  std::vector<ExpressionDescription> arguments;

  friend bool operator==(const ActionDescription & a, const ActionDescription & b) noexcept
  {
    constexpr auto tie = [] (const auto & d) {return std::tie(d.type, d.arguments);};
    return tie(a) == tie(b);
  }
};

using ActionDescriptions = std::vector<ActionDescription>;

inline std::unordered_set<std::string> get_raised_events(const ActionDescriptions & descriptions)
{
  std::unordered_set<std::string> events;
  for (const ActionDescription & ds : descriptions) {
    if (ds.type == "RAISE") {
      events.emplace(std::get<std::string>(ds.arguments.front().arg));
    }
  }
  return events;
}

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_ACTION_DESCRIPTION_HPP
