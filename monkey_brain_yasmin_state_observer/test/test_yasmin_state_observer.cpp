#include <gmock/gmock.h>
#include <gtest/gtest.h>


#include "monkey_brain_yasmin_state_observer/yasmin_state_observer.hpp"

#include "monkey_brain_scxml/state_builder.hpp"
#include "monkey_brain_ros_utils/ros_context.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yasmin_msgs/msg/state_machine.hpp>

using namespace monkey_brain_yasmin_state_observer;
using std::placeholders::_1;

struct ROSEnabledTest : ::testing::Test
{
  ROSEnabledTest() {rclcpp::init(0UL, nullptr);}
  ~ROSEnabledTest() override {rclcpp::shutdown();}
};

struct AYasminStateObserver : ROSEnabledTest
{

  void on_state_message(const yasmin_msgs::msg::StateMachine & msg)
  {
    last_msg = msg;
    signal.set_value();
  }

  void spin()
  {
    rclcpp::executors::spin_node_until_future_complete(
      executor, node.get_node_base_interface(), signal.get_future());
    signal = {};
  }

  yasmin_msgs::msg::StateMachine last_msg;
  std::promise<void> signal;
  rclcpp::Node node{"subscriber"};
  monkey_brain_ros_utils::ROSContext context{&node};
  std::shared_ptr<rclcpp::SubscriptionBase> subscription =
    node.create_subscription<yasmin_msgs::msg::StateMachine>(
    "/fsm_viewer", 1, std::bind(&AYasminStateObserver::on_state_message, this, _1));
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(AYasminStateObserver, publishesSingleStateAsSingleStateWithSubstate) {
  YasminStateObserver obs{&context};
  std::vector<monkey_brain_scxml::State> states;
  states.push_back(monkey_brain_scxml::StateBuilder{"first_state"}.build());
  obs.initialize(states);
  spin();
  ASSERT_EQ(2UL, last_msg.states.size());
  const auto & first_state_msg = last_msg.states.front();
  ASSERT_EQ("monkey_brain", first_state_msg.name);
  const auto & second_state_msg = last_msg.states.back();
  ASSERT_EQ("first_state", second_state_msg.name);
  ASSERT_EQ(first_state_msg.id, second_state_msg.parent);
}
