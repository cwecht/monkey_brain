#include "monkey_brain_scxml/state_machine.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <cassert>
#include <numeric>
#include <sstream>

namespace monkey_brain_scxml
{
namespace
{

template<typename Range, typename T>
bool contains(const Range & rng, const T & value)
{
  return std::end(rng) != std::find(std::begin(rng), std::end(rng), value);
}

State const * find_common_ancestor(State const * a, State const * b)
{
  assert(a != nullptr);
  assert(b != nullptr);
  for (State const * aa = a->parent_state; aa != nullptr; aa = aa->parent_state) {
    for (State const * bb = b->parent_state; bb != nullptr; bb = bb->parent_state) {
      if (aa == bb) {
        return aa;
      }
    }
  }
  return nullptr;
}

State const * find_common_ancestor(
  State const * s,
  const std::vector<State const *> & states)
{
  return std::accumulate(
    states.begin(), states.end(), s,
    [](State const * a, State const * b) {return find_common_ancestor(a, b);});
}

bool is_descendant(State const * s1, State const * s2)
{
  if (s1 == nullptr) {
    return false;
  }
  if (s2 == nullptr) {
    return true;
  }
  for (s1 = s1->parent_state; s1 != nullptr; s1 = s1->parent_state) {
    if (s1 == s2) {return true;}
  }
  return false;
}

std::size_t count_concurrent_substates(const State & parent);

using Reduction = std::size_t (*)(std::size_t, std::size_t);

std::size_t count_concurrent(const std::vector<State> & states, Reduction reduce)
{
  return std::transform_reduce(
    states.begin(),
    states.end(), 0UL, reduce, &count_concurrent_substates);
}

std::size_t count_concurrent_substates(const State & parent)
{
  if (parent.is_history()) {
    return 0UL;
  }
  const auto & states = parent.substates;
  /* *INDENT-OFF* */
  const Reduction reduction = parent.is_parallel()
    ? [] (std::size_t a, std::size_t b) {return a + b;}
    : [] (std::size_t a, std::size_t b) {return std::max(a, b);};
  /* *INDENT-ON* */

  return 1 + count_concurrent(states, reduction);
}

std::size_t count_concurrent_states(const std::vector<State> & states)
{
  const auto max = [](std::size_t a, std::size_t b) {return std::max(a, b);};
  return count_concurrent(states, max);
}

bool is_null_event(const Transition & t)
{
  return t.event.empty();
}

bool select_null_events(const Transition & t, std::string_view)
{
  return is_null_event(t);
}

bool select_matching_event(const Transition & t, std::string_view event)
{
  return t.event == "*" || t.event == event;
}

template<typename CondPtr>
bool holds(const CondPtr & cond)
{
  return !cond || cond->holds();
}

std::size_t count_atomic_substates(const State & s)
{
  if (s.is_atomic()) {
    return 1UL;
  } else {
    const auto & states = s.substates;
    return std::transform_reduce(
      states.begin(),
      states.end(), 0UL, std::plus<>{}, &count_atomic_substates);
  }
}

std::unordered_map<State const *, std::vector<State const *>> preallocate_histories(
  const std::vector<State> & states)
{
  std::unordered_map<State const *, std::vector<State const *>> state_to_history;
  for (const State & s : states) {
    if (s.is_history()) {
      std::vector<State const *> & hist =
        state_to_history.emplace(&s, std::vector<State const *>{}).first->second;
      /* *INDENT-OFF* */
      const std::size_t history_capacity = s.type == State::Type::HISTORY_SHALLOW
              ? s.substates.size()
              : count_atomic_substates(*s.parent_state);
      /* *INDENT-ON* */
      hist.reserve(history_capacity);
    } else {
      state_to_history.merge(preallocate_histories(s.substates));
    }
  }
  return state_to_history;
}

bool has_empty_leave_set(const Transition & t)
{
  return t.target_state.empty();
}

State const * get_history_state_in(State const * state, const StateSet & state_set)
{
  const auto & substates = state->substates;
  const auto it = std::find_if(
    substates.begin(), substates.end(),
    [&state_set](const State & s) {return s.is_history() and state_set.contains(&s);});
  return it != substates.end() ? &(*it) : nullptr;
}

State const * find_final_substate(State const * s)
{
  const auto & substates = s->substates;
  const auto it = std::find_if(substates.begin(), substates.end(), std::mem_fn(&State::is_final));
  return (it == s->substates.end()) ? nullptr : &(*it);
}

bool contains_descendant_of(const StateSet & set, const State & reference)
{
  return std::any_of(
    set.begin(), set.end(), [&reference](State const * s) {
      return is_descendant(s, &reference);
    });
}

} // namespace

std::vector<State const *> & StateMachine::get_history_of(State const * s)
{
  auto h = state_to_history_.find(s);
  assert(h != state_to_history_.end());
  return h->second;
}

State const * StateMachine::find_domain(
  State const * sstate, const std::vector<State const *> & target_states)
{
  State const * domain = sstate;
  if (domain == nullptr) {return domain;}
  for (State const * tstate : target_states) {
    if (tstate->is_history()) {
      auto & history = get_history_of(tstate);

      const auto & nested = history.empty() ?
        tstate->transitions.front().target_state :
        history;
      domain = find_domain(domain, nested);
    } else {
      domain = find_common_ancestor(domain, tstate);
    }
    for (; domain != nullptr and not domain->is_compound(); domain = domain->parent_state) {
    }
  }
  return domain;
}

State const * StateMachine::find_domain(Transition const * transition)
{
  State const * source = transition->source;
  const auto & targets = transition->target_state;
  if (transition->is_internal and source->is_compound() and
    std::all_of(
      targets.begin(), targets.end(),
      [source](State const * s) {return is_descendant(s, source);}))
  {
    return source;
  }
  return find_domain(source, targets);
}

StateMachine::StateMachine(
  std::function<void()> on_shutdown,
  std::unique_ptr<StateMachineObserver> sm_obs,
  const veneer::LoggerPtr & parent_logger)
: logger_{parent_logger->get_child("state_machine")}
  , sm_obs_{std::move(sm_obs)}
  , on_shutdown_{std::move(on_shutdown)}
{
}

void StateMachine::initialize(States states)
{
  states_ = std::move(states.states);
  state_to_history_ = preallocate_histories(states_);

  if (sm_obs_) {
    sm_obs_->initialize(states_);
  }
  preallocate_current_states(count_concurrent_states(states_));

  Transition t{"", nullptr, {}, states.initial_states, false, nullptr};
  selected_transitions_.push_back(&t);

  enter_states(selected_transitions_, state_set_);
  process_internal_events();
}

void StateMachine::post_event(const std::string_view event)
{
  post_event_internal(event, &select_matching_event);
  process_internal_events();
}

void StateMachine::process_internal_events()
{
  const auto select_matcher_and_event = [this]() -> std::pair<std::string_view, event_matcher> {
      if (has_optimal_null_transitions()) {
        return {"", &select_null_events};
      } else if (not internal_events_.empty()) {
        const auto ev = internal_events_.front();
        internal_events_.erase(internal_events_.begin());
        return {ev, &select_matching_event};
      } else {
        return {"", nullptr};
      }
    };

  while (true) {
    const auto [event, matcher] = select_matcher_and_event();
    if (matcher == nullptr) {
      break;
    }
    post_event_internal(event, matcher);
  }

  if (sm_obs_) {
    sm_obs_->on_state_change(active_states());
  }
}

void StateMachine::post_internal_event(const std::string_view event)
{
  internal_events_.push_back(event);
}

bool StateMachine::leave_common_subset_of_states(const Transition & a, const Transition & b) const
{
  if (has_empty_leave_set(a) or has_empty_leave_set(b)) {
    return false;
  }
  State const * common_ancestor_a = find_common_ancestor(
    a.source, a.target_state);
  State const * common_ancestor_b = find_common_ancestor(
    b.source, b.target_state);

  return common_ancestor_a == nullptr or common_ancestor_b == nullptr
         or common_ancestor_a == common_ancestor_b
         or is_descendant(common_ancestor_a, common_ancestor_b)
         or is_descendant(common_ancestor_b, common_ancestor_a);
}

void StateMachine::compute_exit_set(
  const std::vector<Transition const *> & transitions,
  StateSet & exit_set)
{
  exit_set.clear();

  for (Transition const * transition  : transitions) {
    if (transition->target_state.empty()) {
      continue;
    }
    State const * domain = find_domain(transition);
    for (State const * s : active_states_) {
      if (is_descendant(s, domain)) {
        exit_set.insert(s);
      }
    }
  }
}

void StateMachine::exit_states(
  const std::vector<Transition const *> & transitions,
  StateSet & exit_set)
{
  exit_set.clear();
  compute_exit_set(transitions, exit_set);
  // sort by exit order
  for (State const * s : boost::adaptors::reverse(exit_set)) {
    for (const State & child : s->substates) {
      if (child.type == State::Type::HISTORY_SHALLOW) {
        auto & history = get_history_of(&child);
        history.clear();
        for (const State & c : s->substates) {
          if (is_active(&c)) {
            history.push_back(&c);
          }
        }
      } else if (child.type == State::Type::HISTORY_DEEP) {
        auto & history = get_history_of(&child);
        history.clear();
        for (State const * state : active_states_) {
          if (state->is_atomic() and is_descendant(state, s)) {
            history.push_back(state);
          }
        }
      }
    }
    perform_exit_action(s);
  }
  for (State const * s : exit_set) {
    active_states_.erase(s);
  }
}

void StateMachine::execute_transition_actions(const std::vector<Transition const *> & transitions)
{
  for (Transition const * transition : transitions) {
    perform_transition_action(transition);
  }
}

void StateMachine::add_descendant_states_to_enter(
  State const * state, StateSet & states_to_enter,
  StateSet & history_states_via_default)
{
  if (state->is_history()) {
    auto & history = get_history_of(state);

    if (not history.empty()) {
      for (State const * hs : history) {
        add_descendant_states_to_enter(
          hs, states_to_enter, history_states_via_default);
      }
      for (State const * hs : history) {
        add_ancestor_states_to_enter(
          hs, state->parent_state, states_to_enter,
          history_states_via_default);
      }
    } else {
      history_states_via_default.insert(state);
      assert(state->transitions.size() == 1);
      for (State const * is : state->transitions.front().target_state) {
        add_descendant_states_to_enter(
          is, states_to_enter, history_states_via_default);
      }
      for (State const * is : state->transitions.front().target_state) {
        add_ancestor_states_to_enter(
          is, state->parent_state, states_to_enter, history_states_via_default);
      }
    }
  } else {
    states_to_enter.insert(state);
    if (state->is_compound()) {
      for (State const * s : state->automatic_sub_states) {
        add_descendant_states_to_enter(
          s, states_to_enter, history_states_via_default);
      }
      for (State const * s : state->automatic_sub_states) {
        add_ancestor_states_to_enter(
          s, state, states_to_enter, history_states_via_default);
      }
    } else if (state->is_parallel()) {
      add_descendant_states_of_parallel_substates_to_enter(
        state, states_to_enter,
        history_states_via_default);
    }
  }
}

void StateMachine::add_ancestor_states_to_enter(
  State const * state, State const * ancestor,
  StateSet & states_to_enter, StateSet & history_states_via_default)
{
  for (State const * s = state->parent_state; s != ancestor && s != nullptr; s = s->parent_state) {
    states_to_enter.insert(s);
    if (s->is_parallel()) {
      add_descendant_states_of_parallel_substates_to_enter(
        s, states_to_enter,
        history_states_via_default);
    }
  }
}

void StateMachine::add_descendant_states_of_parallel_substates_to_enter(
  State const * parent, StateSet & states_to_enter, StateSet & history_states_via_default)
{
  for (const State & child : parent->substates) {
    if (not child.is_history() and not contains_descendant_of(states_to_enter, child)) {
      add_descendant_states_to_enter(
        &child, states_to_enter, history_states_via_default);
    }
  }
}


void StateMachine::add_ancestors_of_target(
  State const * s, State const * domain, StateSet & states_to_enter,
  StateSet & history_states_via_default)
{
  if (s->is_history()) {
    auto & history = get_history_of(s);
    if (not history.empty()) {
      for (State const * hs : history) {
        add_ancestor_states_to_enter(
          hs, domain, states_to_enter, history_states_via_default);
      }
    } else {
      for (State const * is : s->transitions.front().target_state) {
        add_ancestors_of_target(
          is, domain, states_to_enter, history_states_via_default);
      }
    }
  } else {
    add_ancestor_states_to_enter(
      s, domain, states_to_enter, history_states_via_default);
  }
}

void StateMachine::compute_entry_set(
  const std::vector<Transition const *> & transitions,
  StateSet & states_to_enter, StateSet & history_states_via_default)
{
  for (Transition const * transition : transitions) {
    for (State const * s : transition->target_state) {
      add_descendant_states_to_enter(
        s, states_to_enter, history_states_via_default);
    }
    State const * domain = find_domain(transition);
    for (State const * s : transition->target_state) {
      add_ancestors_of_target(
        s, domain, states_to_enter, history_states_via_default);
    }
  }
}

void StateMachine::enter_states(
  const std::vector<Transition const *> & transitions,
  StateSet & states_to_enter)
{
  states_to_enter.clear();
  history_states_via_default_.clear();
  compute_entry_set(
    transitions, states_to_enter, history_states_via_default_);
  for (State const * state : states_to_enter) {
    active_states_.insert(state);
    perform_entry_action(state);
    if (state->is_compound() and not state->is_history()) {
      execute(state->init_actions);
    }
    if (State const * h = get_history_state_in(state, history_states_via_default_)) {
      perform_transition_action(&(h->transitions.front()));
    }
    if (state->is_final()) {
      post_final_events(state);
    }
  }
#if (VENEER_LOG_MIN_SEVERITY == VENEEER_LOG_SEVERITY_DEBUG)
  constexpr auto to_string = [](std::string_view sv) {return std::string{sv};};
  const std::string active_states = boost::algorithm::join(this->active_states() |
      boost::adaptors::transformed(to_string), ", ");
  VENEER_LOG_DEBUG(logger_, "entered states: %s", active_states.c_str());
#endif
}

void StateMachine::post_event_internal(const std::string_view event, event_matcher event_matches)
{
  selected_transitions_.clear();
  for (State const * old_state : active_states_) {
    if (not old_state->is_atomic()) {
      continue;
    }
    Transition const * transition = find_matching_transition(*old_state, event, event_matches);
    if (transition != nullptr) {
      selected_transitions_.push_back(transition);
    }
  }
  filtered_transitions_.clear();
  for (Transition const * t1 : selected_transitions_) {
    bool preempted = false;
    for (auto & t2 : filtered_transitions_) {
      if (leave_common_subset_of_states(*t1, *t2)) {
        if (is_descendant(t1->source, t2->source)) {
          t2 = nullptr;
        } else {
          preempted = true;
          break;
        }
      }
    }
    if (not preempted) {
      filtered_transitions_.erase(
        std::remove(
          filtered_transitions_.begin(),
          filtered_transitions_.end(), nullptr), filtered_transitions_.end());
      filtered_transitions_.push_back(t1);
    }
  }
  std::swap(filtered_transitions_, selected_transitions_);
  // deduplicate
  filtered_transitions_.clear();
  for (Transition const * t : selected_transitions_) {
    if (not contains(filtered_transitions_, t)) {
      filtered_transitions_.push_back(t);
    }
  }

  exit_states(filtered_transitions_, state_set_);

  execute_transition_actions(filtered_transitions_);

  enter_states(filtered_transitions_, state_set_);
}

void StateMachine::post_final_events(State const * state)
{
  const auto parent_state = state->parent_state;
  if (parent_state == nullptr) {
    if (on_shutdown_) {
      on_shutdown_();
    }
  } else {
    post_internal_event(parent_state->done_event);

    const auto grand_parent = parent_state->parent_state;
    if (grand_parent != nullptr && all_substates_finalized(grand_parent)) {
      post_internal_event(grand_parent->done_event);
    }
  }
}

Transition const * StateMachine::find_matching_transition(
  const State & state, std::string_view event, event_matcher event_matches) const
{
  const auto & transitions = state.transitions;
  auto transition = std::find_if(
    transitions.begin(), transitions.end(), [event_matches, event, this](const auto & t) {
      return event_matches(t, event) && holds(t.condition);
    });

  if (transition != transitions.end()) {
    return &(*transition);
  } else if (state.parent_state == nullptr) {
    return nullptr;
  } else {
    return find_matching_transition(*state.parent_state, event, event_matches);
  }
}

void StateMachine::perform_exit_action(const State * state)
{
  execute(state->on_exit_action);

  const auto & transitions = state->transitions;
  for (auto it = transitions.begin(); it != transitions.end(); ++it) {
    if (is_null_event(*it)) {
      null_preselected_transitions_.erase(
        std::remove_if(
          null_preselected_transitions_.begin(), null_preselected_transitions_.end(),
          [t = &(*it)](auto * e) {return e == t;}),
        null_preselected_transitions_.end());
    }
  }
}

void StateMachine::perform_entry_action(const State * state)
{
  const auto & transitions = state->transitions;
  for (auto it = transitions.begin(); it != transitions.end(); ++it) {
    if (is_null_event(*it)) {
      null_preselected_transitions_.push_back(&(*it));
    }
  }
  execute(state->on_entry_action);
}

void StateMachine::perform_transition_action(const Transition * transition)
{
  execute(transition->action);
}

bool StateMachine::has_optimal_null_transitions() const
{
  return std::any_of(
    null_preselected_transitions_.begin(), null_preselected_transitions_.end(),
    [this](auto * e) {return holds(e->condition);});
}

void StateMachine::preallocate_current_states(std::size_t capacity)
{
  active_states_.reserve(capacity);
  state_set_.reserve(capacity);
  // TODO(cwecht) use number of history states
  history_states_via_default_.reserve(capacity);
  // TODO(cwecht) use number of transitions without event
  null_preselected_transitions_.reserve(capacity);
  // at most one transition can be selected per state
  selected_transitions_.reserve(capacity);
  filtered_transitions_.reserve(selected_transitions_.size());
}

bool StateMachine::is_finalized(State const * s) const
{
  State const * const final_substate = find_final_substate(s);
  return (final_substate == nullptr) ? false : is_active(final_substate);
}

bool StateMachine::is_active(State const * s) const
{
  return active_states_.contains(s);
}

bool StateMachine::all_substates_finalized(State const * s) const
{
  const auto is_finalized = [this](const State & state) {
      return this->is_finalized(&state);
    };
  return std::all_of(s->substates.begin(), s->substates.end(), is_finalized);
}

} // namespace monkey_brain_scxml
