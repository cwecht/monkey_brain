#include "monkey_brain_scxml/build_states.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/range/combine.hpp>

namespace monkey_brain_scxml
{
namespace
{

Transition build_transition(
  const TransitionDescription & desc,
  const monkey_brain_core::OperationsFactory & ops_factory)
{
  return Transition{
    desc.event,
    desc.condition ? ops_factory.create_condition(*desc.condition) : nullptr,
    ops_factory.create_actions(desc.action),
    {},
    desc.type == "internal",
    nullptr
  };
}

std::vector<Transition> build_transitions(
  const std::vector<TransitionDescription> & transition_descriptions,
  const monkey_brain_core::OperationsFactory & ops_factory)
{
  std::vector<Transition> transitions;
  transitions.reserve(transition_descriptions.size());
  for (auto const & td : transition_descriptions) {
    transitions.push_back(build_transition(td, ops_factory));
  }
  return transitions;
}

State const * find_state(const std::vector<State> & states, const std::string & name)
{
  for (auto & state : states) {
    if (state.name == name) {
      return &state;
    } else {
      State const * found = find_state(state.substates, name);
      if (found != nullptr) {
        return found;
      }
    }
  }
  return nullptr;
}

std::vector<State const *> find_states(const std::vector<State> & states, const std::string & name)
{
  std::vector<std::string> sds;
  boost::split(sds, name, boost::is_any_of("\t "), boost::token_compress_on);
  std::vector<State const *> targets;
  targets.reserve(sds.size());
  for (const std::string & sd : sds) {
    State const * target = find_state(states, sd);
    if (target == nullptr) {
      throw std::runtime_error("state '" + name + "' not found!");
    }
    targets.push_back(target);
  }
  return targets;
}


std::vector<State const *> find_initial_state(
  const std::vector<State> & states,
  const std::string & name)
{
  if (name.empty()) {
    return {&states.front()};
  }
  return find_states(states, name);
}


const StateDescription * look_up(
  const std::vector<StateDescription> & state_descriptions,
  const std::string & name)
{
  for (const auto & sd : state_descriptions) {
    if (sd.name == name) {
      return &sd;
    } else if (sd.nested) {
      auto * found = look_up(sd.nested->states, name);
      if (found != nullptr) {
        return found;
      }
    }
  }
  return nullptr;
}

std::vector<State> build_states_impl(
  const std::vector<StateDescription> & state_descriptions,
  const monkey_brain_core::OperationsFactory & ops_factory);

State::Type to_type(StateType type)
{
  switch (type) {
    case StateType::FINAL:
      return State::Type::FINAL;
    case StateType::HISTORY_SHALLOW:
      return State::Type::HISTORY_SHALLOW;
    case StateType::HISTORY_DEEP:
      return State::Type::HISTORY_DEEP;
    case StateType::REGULAR:
    default:
      return State::Type::REGULAR;
  }
}

State build_state(
  const StateDescription & sd,
  const monkey_brain_core::OperationsFactory & ops_factory)
{
  std::vector<State> substates;
  std::vector<State const *> initial;
  monkey_brain_core::Actions init_actions;
  if (sd.nested) {
    substates = build_states_impl(sd.nested->states, ops_factory);
    if (not sd.nested->are_parallel) {
      initial = find_initial_state(substates, sd.nested->initial.state);
      init_actions = ops_factory.create_actions(sd.nested->initial.action);
    }
  }
  return {sd.name,
    build_transitions(sd.transitions, ops_factory),
    ops_factory.create_actions(sd.on_entry_action),
    ops_factory.create_actions(sd.on_exit_action),
    std::move(substates),
    std::move(initial),
    std::move(init_actions),
    nullptr, // parent is initialized later in build_states_impl
    to_type(sd.type),
    "done.state." + sd.name
  };
}

std::vector<State> build_states_impl(
  const std::vector<StateDescription> & state_descriptions,
  const monkey_brain_core::OperationsFactory & ops_factory)
{
  std::vector<State> states;
  states.reserve(state_descriptions.size());
  for (auto const & sd : state_descriptions) {
    states.push_back(build_state(sd, ops_factory));
    for (State & sub : states.back().substates) {
      sub.parent_state = &states.back();
    }
    for (Transition & t : states.back().transitions) {
      t.source = &states.back();
    }
  }
  return states;
}

//void check_duplicated_state_ids(const std::vector<StateDescription>& sds) {
//  std::vector<std::string_view> names;
//  names.reserve(sds.size());
//  for (const auto& sd : sds) {
//    names.push_back(sd.name);
//  }
//  std::sort(names.begin(), names.end());
//  auto id = std::adjacent_find(names.begin(), names.end());
//  if (id != names.end()) {
//    throw std::runtime_error("Duplicate state id: '" + std::string{*id} + "'");
//  }
//}

void wire_up_transitions(
  const std::vector<StateDescription> & state_descriptions,
  std::vector<State> & all_states,
  std::vector<State> & current_states)
{
  for (auto & s : current_states) {
    wire_up_transitions(state_descriptions, all_states, s.substates);

    auto * d = look_up(state_descriptions, s.name);
    using boost::combine;
    for (auto [transition, desc] : combine(s.transitions, d->transitions)) {
      if (desc.target.empty()) {
        continue;
      }
      transition.target_state = find_states(all_states, desc.target);
    }
  }
}


std::size_t assign_document_id(std::vector<State> & states, std::size_t id = 0)
{
  for (auto & s : states) {
    s.document_order_id = id;
    id = assign_document_id(s.substates, id + 1);
  }
  return id;
}

} // namespace

States build_states(
  const StateMachineDescription & state_machine_description,
  const monkey_brain_core::OperationsFactory & ops_factory)
{
  const auto & state_descriptions = state_machine_description.states;
  auto states = build_states_impl(state_descriptions, ops_factory);
  if (states.empty()) {
    throw std::runtime_error("State Machine has no states!");
  }
  assign_document_id(states);
  wire_up_transitions(state_descriptions, states, states);
  std::vector<State const *> initial_states = find_initial_state(
    states,
    state_machine_description.initial.state);

  auto init_actions = ops_factory.create_actions(state_machine_description.initial.action);

  return {std::move(states), std::move(initial_states), std::move(init_actions)};
}

} // namespace monkey_brain_scxml
