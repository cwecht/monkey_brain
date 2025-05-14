#include "monkey_brain_scxml/validate_state_machine.hpp"

#include <algorithm>
#include <functional>
#include <unordered_set>
#include <string>

namespace mbc = monkey_brain_core;

namespace monkey_brain_scxml
{
namespace
{

std::unordered_set<std::string> get_defined_events(const TransitionDescription & td)
{
  return mbc::get_raised_events(td.action);
}

std::unordered_set<std::string> get_defined_events(const std::vector<TransitionDescription> & ds)
{
  std::unordered_set<std::string> events;
  for (const auto & d : ds) {
    events.merge(get_defined_events(d));
  }
  return events;
}

std::unordered_set<std::string> get_defined_events(const StateDescription & sd);

template<typename Rng, typename T>
bool any_of(const Rng & rng, const T & val)
{
  return std::any_of(std::begin(rng), std::end(rng), val);
}

bool has_final_child_or_grand_childs(const StateDescription & sd)
{
  if (sd.nested == nullptr) {
    return false;
  }
  const auto & states = sd.nested->states;
  return any_of(states,
           [] (const auto & s) {
             return s.is_final() or
                    (s.nested != nullptr && any_of(s.nested->states,
          std::mem_fn(&StateDescription::is_final)));
                                                                                                          }
  );
}

std::unordered_set<std::string> get_defined_events(const std::vector<StateDescription> & ds)
{
  std::unordered_set<std::string> events;
  for (const auto & d : ds) {
    events.merge(get_defined_events(d));
    if (has_final_child_or_grand_childs(d)) {
      events.emplace("done.state." + d.name);
    }
  }
  return events;
}

std::unordered_set<std::string> get_defined_events(const StateDescription & sd)
{
  std::unordered_set<std::string> events;
  events.merge(mbc::get_raised_events(sd.on_entry_action));
  events.merge(mbc::get_raised_events(sd.on_exit_action));
  events.merge(get_defined_events(sd.transitions));

  if (sd.nested != nullptr) {
    events.merge(get_defined_events(sd.nested->states));
    events.merge(mbc::get_raised_events(sd.nested->initial.action));
  }
  if (sd.is_final()) {

  }
  return events;
}

std::unordered_set<std::string> get_defined_events(const StateMachineDescription & smd)
{
  std::unordered_set<std::string> events = mbc::get_raised_events(smd.initial.action);
  events.merge(get_defined_events(smd.states));
  return events;
}

std::unordered_set<std::string> get_referenced_events(const TransitionDescription & td)
{
  std::unordered_set<std::string> events;
  events.emplace(td.event);
  return events;
}

std::unordered_set<std::string> get_referenced_events(const StateDescription & sd);

template<typename T>
std::unordered_set<std::string> get_referenced_events(const std::vector<T> & ds)
{
  std::unordered_set<std::string> events;
  for (const auto & d : ds) {
    events.merge(get_referenced_events(d));
  }
  return events;
}

std::unordered_set<std::string> get_referenced_events(const StateDescription & sd)
{
  std::unordered_set<std::string> events;
  events.merge(get_referenced_events(sd.transitions));
  if (sd.nested != nullptr) {
    events.merge(get_referenced_events(sd.nested->states));
  }
  return events;
}

std::unordered_set<std::string> get_referenced_events(const StateMachineDescription & smd)
{
  return get_referenced_events(smd.states);
}

template<typename T>
bool contains(const std::unordered_set<T> & rng, const T & to_find)
{
  return rng.end() != rng.find(to_find);
}

std::unordered_set<std::string> gather_defined_events(
  const StateMachineDescription & sds,
  const monkey_brain_core::Environment & env)
{
  auto all_defined_events = env.get_all_events();
  all_defined_events.merge(get_defined_events(sds));
  return all_defined_events;
}

ValidationErrors validate_referenced_events(
  const std::unordered_set<std::string> & defined_events,
  const std::unordered_set<std::string> & referenced_events)
{
  ValidationErrors errors;
  for (const auto & target : referenced_events) {
    if (not contains(defined_events, target)) {
      errors.push_back({"there is no event for target: " + target});
    }
  }
  return errors;
}

} // namespace

ValidationErrors validate_state_machine(
  const StateMachineDescription & sds,
  const monkey_brain_core::Environment & env)
{
  auto defined_events = gather_defined_events(sds, env);
  auto referenced_events = get_referenced_events(sds);
  referenced_events.erase("*");
  referenced_events.erase("");

  return validate_referenced_events(defined_events, referenced_events);
}

} // namespace monkey_brain_scxml
