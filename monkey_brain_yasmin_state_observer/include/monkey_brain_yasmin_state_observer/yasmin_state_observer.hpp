#ifndef MONKEY_BRAIN_YASMIN_STATE_OBSERVER_HPP
#define MONKEY_BRAIN_YASMIN_STATE_OBSERVER_HPP

#include "monkey_brain_core/context.hpp"
#include "monkey_brain_scxml/state_machine_observer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yasmin_msgs/msg/state_machine.hpp>

namespace monkey_brain_yasmin_state_observer
{

class YasminStateObserver : public monkey_brain_scxml::StateMachineObserver
{
public:
  explicit YasminStateObserver(monkey_brain_core::Context * context);
  void initialize(const std::vector<monkey_brain_scxml::State> & states) final;
  void on_state_change(monkey_brain_scxml::StateNameRange currently_active_states) final;

private:
  using StatesMap = std::map<monkey_brain_scxml::State const *, std::int32_t>;
  void add_as_msg(const monkey_brain_scxml::State & s, const StatesMap & states_to_ids);
  void add_as_msg(
    const std::vector<monkey_brain_scxml::State> & states,
    const StatesMap & states_to_ids);
  std::shared_ptr<rclcpp::Publisher<yasmin_msgs::msg::StateMachine>> publisher_;
  yasmin_msgs::msg::StateMachine sm_msg_;
};

} // namespace monkey_brain_yasmin_state_observer
#endif // MONKEY_BRAIN_YASMIN_STATE_OBSERVER_HPP
