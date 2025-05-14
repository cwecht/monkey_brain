#include "monkey_brain_yasmin_state_observer/yasmin_state_observer.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace monkey_brain_yasmin_state_observer
{
namespace
{
using StatesMap = std::map<monkey_brain_scxml::State const *, std::int32_t>;

void add_to_states_map(
  const std::vector<monkey_brain_scxml::State> & states, StatesMap & map,
  std::int32_t & next_free_id);

void add_to_states_map(
  const monkey_brain_scxml::State & s, StatesMap & map,
  std::int32_t & next_free_id)
{
  map[&s] = next_free_id++;
  add_to_states_map(s.substates, map, next_free_id);
}
void add_to_states_map(
  const std::vector<monkey_brain_scxml::State> & states, StatesMap & map,
  std::int32_t & next_free_id)
{
  for (const monkey_brain_scxml::State & s : states) {
    add_to_states_map(s, map, next_free_id);
  }
}

std::map<monkey_brain_scxml::State const *, std::int32_t> to_state_map(
  const std::vector<monkey_brain_scxml::State> & states)
{
  StatesMap map;
  std::int32_t id = 1;
  add_to_states_map(states, map, id);
  return map;
}
}

YasminStateObserver::YasminStateObserver(monkey_brain_core::Context * context)
: publisher_{
    context->get_concrete_context<rclcpp::Node>()->create_publisher<yasmin_msgs::msg::StateMachine>(
      "/fsm_viewer", 1)}
{}

void
YasminStateObserver::add_as_msg(
  const std::vector<monkey_brain_scxml::State> & states,
  const StatesMap & states_to_ids)
{
  for (const monkey_brain_scxml::State & s : states) {
    add_as_msg(s, states_to_ids);
  }
}

void
YasminStateObserver::add_as_msg(
  const monkey_brain_scxml::State & s,
  const StatesMap & states_to_ids)
{
  yasmin_msgs::msg::State msg;
  msg.name = s.name;
  msg.id = states_to_ids.at(&s);
  msg.parent = (s.parent_state == nullptr) ? 0 : states_to_ids.at(s.parent_state);
  msg.is_fsm = not s.substates.empty();
  for (const monkey_brain_scxml::Transition & t : s.transitions) {
    for (const monkey_brain_scxml::State * target : t.target_state) {
      yasmin_msgs::msg::Transition transition;
      transition.state = target->name;
      transition.outcome = t.event;
      msg.transitions.push_back(transition);
      msg.outcomes.push_back(transition.outcome);
    }
  }
  sm_msg_.states.push_back(msg);
  add_as_msg(s.substates, states_to_ids);
}

void
YasminStateObserver::initialize(const std::vector<monkey_brain_scxml::State> & states)
{
  const auto states_to_ids = to_state_map(states);
  yasmin_msgs::msg::State msg;
  msg.name = "monkey_brain";
  msg.is_fsm = true;
  sm_msg_.states.push_back(msg);

  add_as_msg(states, states_to_ids);
  publisher_->publish(sm_msg_);
}

void
YasminStateObserver::on_state_change(monkey_brain_scxml::StateNameRange currently_active_states)
{
  for (auto & s : sm_msg_.states) {
    s.current_state = -1;
  }
  for (std::string_view name : currently_active_states) {
    const auto it = std::find_if(
      std::begin(sm_msg_.states), std::end(sm_msg_.states),
      [name](const auto & s) {return name == s.name;});
    assert(it != std::end(sm_msg_.states));
    const auto parent = it->parent;
    const auto pit = std::find_if(
      std::begin(sm_msg_.states), std::end(sm_msg_.states),
      [parent](const auto & s) {return parent == s.id;});
    pit->current_state = it->id;

  }
  publisher_->publish(sm_msg_);
}

class YasminStateObserverFactory : public monkey_brain_scxml::StateMachineObserverFactory
{
public:
  std::unique_ptr<monkey_brain_scxml::StateMachineObserver> create(
    monkey_brain_core::Context * context) const override
  {
    return std::make_unique<YasminStateObserver>(context);
  }
};

} // namespace monkey_brain_yasmin_state_observer

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_yasmin_state_observer::YasminStateObserverFactory,
  monkey_brain_scxml::StateMachineObserverFactory)
