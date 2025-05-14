@{
from rosidl_generator_monkey_brain import to_upper_case
from rosidl_generator_monkey_brain import create_reference_name
from rosidl_generator_monkey_brain import create_references
from rosidl_generator_monkey_brain import create_full_type_decriptors
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
request_message, response_message = service.request_message, service.response_message
plugin_type_name = service.namespaced_type.name + 'ServiceClient'
service_typename = '::'.join(service.namespaced_type.namespaced_name())
response_message_typename = '::'.join(response_message.structure.namespaced_type.namespaced_name())
request_message_typename = '::'.join(request_message.structure.namespaced_type.namespaced_name())
}@
/**
 * Copyright 2024 Christopher Wecht
 */
#include "@('/'.join(map(convert_camel_case_to_lower_case_underscore, service.namespaced_type.namespaced_name())) + '.hpp')"

#include <yaml-cpp/yaml.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>

#include "monkey_brain_core/io_plugin_factory.hpp"
#include "monkey_brain_ros_utils/simple_service_client.hpp"

#include <chrono>

using std::placeholders::_1;

namespace mbc = monkey_brain_core;

class @(plugin_type_name) : public mbc::IOPlugin
{
public:
  @(plugin_type_name)(std::any node, std::string topic_name, const YAML::Node & params)
  : client_{node.has_value() ? monkey_brain_ros_utils::create_simple_client<@(service_typename)>(
        std::any_cast<rclcpp::Node *>(node),
        topic_name,
        std::bind(&@(plugin_type_name)::on_response, this, _1)) : nullptr}
    , timer_{node.has_value() ? std::any_cast<rclcpp::Node *>(node)->create_timer(
        std::chrono::milliseconds{params["retry_period"].as<uint64_t>()},
        std::bind(&@(plugin_type_name)::on_timer, this)) : nullptr}
    , is_ready_{client_ ? client_->service_is_ready() : false}
    , ROOT{topic_name}  {}

  mbc::TypedReferences get_references() const final
  {
    return { {IS_READY_REFERENCE, mbc::ValueTypes::BOOL, mbc::AccessMode::READONLY},
            @(', '.join(create_references(response_message.structure.members, 'mbc::AccessMode::READONLY', 'RESPONSE'))),
            @(', '.join(create_references(request_message.structure.members, 'mbc::AccessMode::WRITEONLY', 'REQUEST')))};
  }

  void const * get_value_handle(std::string_view const ref) const
  {
    if (ref == IS_READY_REFERENCE) {
      return &is_ready_;
@[for descriptor in create_full_type_decriptors(response_message.structure.members)]@
    } else if (ref == RESPONSE_@(to_upper_case('_'.join(descriptor.full_path)))) {
      return &last_response_msg_.@('.'.join(descriptor.full_path));
@[end for]@
    } else {
      return nullptr;
    }
  }

  void assign_value([[maybe_unused]] std::string_view const ref,
                    [[maybe_unused]] void const * ptr) final
  {
    if (false) {
@[for descriptor in create_full_type_decriptors(request_message.structure.members)]@
    } else if (ref == REQUEST_@(to_upper_case('_'.join(descriptor.full_path)))) {
      assign_helper(last_request_msg_.@('.'.join(descriptor.full_path)), ptr);
@[end for]@
    }
  }

  void perform(std::string_view const) final {
    client_->async_send_request(last_request_msg_);
  }

  std::vector<std::string> get_performance_references() const final {
    return {ROOT + "/call"};
  }

  std::vector<std::string> get_events() const final {
    return {RESPONSE_ROOT, IS_READY_REFERENCE};
  }

private:
  void on_response(const @(response_message_typename) & msg)
  {
    last_response_msg_ = msg;
    post_event(RESPONSE_ROOT);
  }

  void on_timer()
  {
    is_ready_ = client_->service_is_ready();
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

  monkey_brain_ros_utils::SimpleClient<@(service_typename)>::SharedPtr client_;
  @(response_message_typename) last_response_msg_;
  @(request_message_typename) last_request_msg_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool is_ready_;

  const std::string ROOT;
  const std::string IS_READY_REFERENCE{ROOT + "/is_ready"};
  const std::string REQUEST_ROOT{ROOT + "/request"};

@[for descriptor in create_full_type_decriptors(request_message.structure.members)]@
  const std::string REQUEST_@(create_reference_name(descriptor)){REQUEST_ROOT + "/@('/'.join(descriptor.full_path))"};
@[end for]@

  const std::string RESPONSE_ROOT{ROOT + "/response"};
@[for descriptor in create_full_type_decriptors(response_message.structure.members)]@
  const std::string RESPONSE_@(create_reference_name(descriptor)){RESPONSE_ROOT + "/@('/'.join(descriptor.full_path))"};
@[end for]@
};

template class monkey_brain_core::IOPluginFactoryImpl<@(plugin_type_name)>;

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_core::IOPluginFactoryImpl<@(plugin_type_name)>,
  monkey_brain_core::IOPluginFactory)
