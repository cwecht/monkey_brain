@{
from rosidl_generator_monkey_brain import to_upper_case
from rosidl_generator_monkey_brain import create_reference_name
from rosidl_generator_monkey_brain import create_references
from rosidl_generator_monkey_brain import create_full_type_decriptors
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
goal_message, result_message, feedback_message = action.goal, action.result, action.feedback
plugin_type_name = action.namespaced_type.name + 'ActionClient'
action_typename = '::'.join(action.namespaced_type.namespaced_name())
goal_message_typename = '::'.join(goal_message.structure.namespaced_type.namespaced_name())
result_message_typename = '::'.join(result_message.structure.namespaced_type.namespaced_name())
feedback_message_typename = '::'.join(feedback_message.structure.namespaced_type.namespaced_name())
}@
/**
 * Copyright 2024 Christopher Wecht
 */
#include "@('/'.join(map(convert_camel_case_to_lower_case_underscore, action.namespaced_type.namespaced_name())) + '.hpp')"

#include <yaml-cpp/yaml.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/create_client.hpp>

#include "monkey_brain_core/io_plugin_factory.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace mbc = monkey_brain_core;

class @(plugin_type_name) : public mbc::IOPlugin
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<@(action_typename)>;
  @(plugin_type_name)(std::any node, std::string action_name, const YAML::Node & params)
  : client_{node.has_value() ? rclcpp_action::create_client<@(action_typename)>(
        std::any_cast<rclcpp::Node *>(node),
        action_name) : nullptr}
    , timer_{node.has_value() ? std::any_cast<rclcpp::Node *>(node)->create_timer(
        std::chrono::milliseconds{params["retry_period"].as<uint64_t>()},
        std::bind(&@(plugin_type_name)::on_timer, this)) : nullptr}
    , is_ready_{client_ ? client_->action_server_is_ready() : false}
    , ROOT{action_name}  {}

  mbc::TypedReferences get_references() const final
  {
    return { {IS_READY_REFERENCE, mbc::ValueTypes::BOOL, mbc::AccessMode::READONLY},
            @(', '.join(create_references(goal_message.structure.members, 'mbc::AccessMode::WRITEONLY', 'GOAL'))),
            @(', '.join(create_references(feedback_message.structure.members, 'mbc::AccessMode::READONLY', 'FEEDBACK'))),
             {RESULT_CODE, mbc::ValueTypes::INT8, mbc::AccessMode::READONLY},
            @(', '.join(create_references(result_message.structure.members, 'mbc::AccessMode::READONLY', 'RESULT')))};
  }

  void const * get_value_handle(std::string_view const ref) const
  {
    if (ref == IS_READY_REFERENCE) {
      return &is_ready_;
    } if (ref == RESULT_CODE) {
      return &result_code_;
@[for descriptor in create_full_type_decriptors(feedback_message.structure.members)]@
    } else if (ref == FEEDBACK_@(to_upper_case('_'.join(descriptor.full_path)))) {
      return &last_feedback_msg_.@('.'.join(descriptor.full_path));
@[end for]@
@[for descriptor in create_full_type_decriptors(result_message.structure.members)]@
    } else if (ref == RESULT_@(to_upper_case('_'.join(descriptor.full_path)))) {
      return &last_result_msg_.@('.'.join(descriptor.full_path));
@[end for]@
    } else {
      return nullptr;
    }
  }

  void assign_value([[maybe_unused]] std::string_view const ref,
                    [[maybe_unused]] void const * ptr) final
  {
    if (false) {
@[for descriptor in create_full_type_decriptors(goal_message.structure.members)]@
    } else if (ref == GOAL_@(to_upper_case('_'.join(descriptor.full_path)))) {
      assign_helper(last_goal_msg_.@('.'.join(descriptor.full_path)), ptr);
@[end for]@
    }
  }

  void perform(std::string_view const) final {
    auto send_goal_options = rclcpp_action::Client<@(action_typename)>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&@(plugin_type_name)::on_response, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&@(plugin_type_name)::on_feedback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&@(plugin_type_name)::on_result, this, _1);
    client_->async_send_goal(last_goal_msg_, send_goal_options);
  }

  std::vector<std::string> get_performance_references() const final {
    return {ROOT + "/send_goal"};
  }

  std::vector<std::string> get_events() const final {
    return {FEEDBACK_ROOT, RESULT_ROOT, IS_READY_REFERENCE};
  }

private:
  void on_response(const GoalHandle::SharedPtr & /*goal_handle*/)
  {
  }

  void on_feedback(GoalHandle::SharedPtr /*goal_handle*/,
                   const std::shared_ptr<const @(feedback_message_typename)> & msg)
  {
    last_feedback_msg_ = *msg;
    post_event(FEEDBACK_ROOT);
  }

  void on_result(const GoalHandle::WrappedResult & result)
  {
    result_code_ = result.code;
    if (result.result != nullptr) {
      last_result_msg_ = *result.result;
    }
    post_event(RESULT_ROOT);
  }

  void on_timer()
  {
    is_ready_ = client_->action_server_is_ready();
    if (is_ready_) {
      timer_->cancel();
      post_event(IS_READY_REFERENCE);
    }
  }

  template<typename Value>
  static void assign_helper(Value & target, void const * source)
  {
    target = *reinterpret_cast<Value const *>(source);
  }

  rclcpp_action::Client<@(action_typename)>::SharedPtr client_;
  @(goal_message_typename) last_goal_msg_;
  rclcpp_action::ResultCode result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  @(result_message_typename) last_result_msg_;
  @(feedback_message_typename) last_feedback_msg_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool is_ready_;

  const std::string ROOT;
  const std::string IS_READY_REFERENCE{ROOT + "/is_ready"};
  const std::string GOAL_ROOT{ROOT + "/goal"};

@[for descriptor in create_full_type_decriptors(goal_message.structure.members)]@
  const std::string GOAL_@(create_reference_name(descriptor)){GOAL_ROOT + "/@('/'.join(descriptor.full_path))"};
@[end for]@

  const std::string RESULT_ROOT{ROOT + "/result"};
  const std::string RESULT_CODE{ROOT + "/result_code"};
@[for descriptor in create_full_type_decriptors(result_message.structure.members)]@
  const std::string RESULT_@(create_reference_name(descriptor)){RESULT_ROOT + "/@('/'.join(descriptor.full_path))"};
@[end for]@

  const std::string FEEDBACK_ROOT{ROOT + "/feedback"};
@[for descriptor in create_full_type_decriptors(feedback_message.structure.members)]@
  const std::string FEEDBACK_@(create_reference_name(descriptor)){FEEDBACK_ROOT + "/@('/'.join(descriptor.full_path))"};
@[end for]@
};

template class monkey_brain_core::IOPluginFactoryImpl<@(plugin_type_name)>;

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_core::IOPluginFactoryImpl<@(plugin_type_name)>,
  monkey_brain_core::IOPluginFactory)
