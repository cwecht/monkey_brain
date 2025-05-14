#ifndef MONKEY_BRAIN_STATE_MACHINE_HPP
#define MONKEY_BRAIN_STATE_MACHINE_HPP

#include "monkey_brain_scxml/states.hpp"
#include "monkey_brain_scxml/state_name_range.hpp"
#include "monkey_brain_scxml/state_machine_observer.hpp"

#include "monkey_brain_core/event_recipient.hpp"
#include "monkey_brain_core/decision_engine.hpp"

#include "veneer/veneer.hpp"

#include <boost/container/flat_set.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <typeinfo>

namespace monkey_brain_scxml
{

struct DocumentOrder
{
  bool operator()(State const * lhs, State const * rhs) const
  {
    return lhs->document_order_id < rhs->document_order_id;
  }
};

using StateSet = boost::container::flat_set<State const *, DocumentOrder>;

class StateMachine : public monkey_brain_core::DecisionEngine
{
public:
  StateMachine(
    std::function<void()> on_shutdown = {},
    std::unique_ptr<StateMachineObserver> sm_obs = nullptr,
    const veneer::LoggerPtr & parent_logger = veneer::ZeroLogger::get_instance());

  void initialize(States states);

  void post_event(const std::string_view event) final;
  void post_internal_event(const std::string_view event) final;
  StateNameRange active_states() const
  {
    return StateNameRange{active_states_};
  }

private:
  using event_matcher = bool (*)(const Transition &, std::string_view);
  void post_final_events(State const * state);
  void process_internal_events();
  void process_optimal_null_transitions();
  void post_event_internal(const std::string_view event, event_matcher event_matches);
  Transition const * find_matching_transition(
    const State & state,
    std::string_view event, event_matcher event_matches) const;
  bool leave_common_subset_of_states(
    const Transition & a,
    const Transition & b) const;

  void compute_exit_set(
    const std::vector<Transition const *> & transitions,
    StateSet & exit_set);
  void exit_states(
    const std::vector<Transition const *> & transitions,
    StateSet & exit_set);
  void execute_transition_actions(const std::vector<Transition const *> & transitions);

  void enter_states(
    const std::vector<Transition const *> & transitions,
    StateSet & states_to_enter);

  void compute_entry_set(
    const std::vector<Transition const *> & transitions,
    StateSet & states_to_enter, StateSet & history_states_via_default);

  void add_ancestor_states_to_enter(
    State const * state, State const * ancestor,
    StateSet & states_to_enter, StateSet & history_states_via_default);
  void add_descendant_states_to_enter(
    State const * state, StateSet & states_to_enter,
    StateSet & history_states_via_default);
  void add_ancestors_of_target(
    State const * s, State const * domain, StateSet & states_to_enter,
    StateSet & history_states_via_default);
  void add_descendant_states_of_parallel_substates_to_enter(
    State const * parent, StateSet & states_to_enter, StateSet & history_states_via_default);

  std::vector<State const *> & get_history_of(State const * s);

  State const * find_domain(
    State const * sstate, const std::vector<State const *> & target_states);
  State const * find_domain(Transition const * transition);

  void perform_entry_action(const State * state);
  void perform_exit_action(const State * state);
  void perform_transition_action(const Transition * transition);
  bool has_optimal_null_transitions() const;
  void preallocate_current_states(std::size_t capacity);
  bool is_finalized(State const * s) const;
  bool is_active(State const * s) const;
  bool all_substates_finalized(State const * s) const;
  void update_states();

  const veneer::LoggerPtr logger_;
  std::unique_ptr<StateMachineObserver> sm_obs_;
  std::function<void()> on_shutdown_;
  std::vector<State> states_;
  StateSet active_states_;
  StateSet state_set_;
  StateSet history_states_via_default_;
  std::vector<std::string_view> internal_events_;
  std::vector<Transition const *> null_preselected_transitions_;
  std::vector<Transition const *> selected_transitions_;
  std::vector<Transition const *> filtered_transitions_;
  std::unordered_map<State const *, std::vector<State const *>> state_to_history_;
};

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_STATE_MACHINE_HPP
