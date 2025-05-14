@{
from rosidl_generator_monkey_brain import to_upper_case
from rosidl_generator_monkey_brain import create_reference_name
from rosidl_generator_monkey_brain import create_references
from rosidl_generator_monkey_brain import create_full_type_decriptors
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
plugin_type_name = message.structure.namespaced_type.name + 'SubscriberPlugin'
type_name = message.structure.namespaced_type.namespaced_name()[-1]
message_typename = '::'.join(message.structure.namespaced_type.namespaced_name())
}@
/**
 * Copyright 2024 Christopher Wecht
 */
// NOLINTNEXTLINE(build/include_order)
#include "@('/'.join(map(convert_camel_case_to_lower_case_underscore, message.structure.namespaced_type.namespaced_name())) + '.hpp')"  // NOLINT(build/include)

// NOLINTNEXTLINE(build/include_order)
#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include)
// NOLINTNEXTLINE(build/include_order)
#include <rclcpp/node.hpp>  // NOLINT(build/include)

// NOLINTNEXTLINE(build/include_order)
#include "monkey_brain_core/io_plugin_factory.hpp"  // NOLINT(build/include)

using std::placeholders::_1;

namespace mbc = monkey_brain_core;

class @(plugin_type_name) : public mbc::IOPlugin
{
public:
  @(plugin_type_name)(std::any node, std::string topic_name, const YAML::Node &)
  : subscription_{node.has_value() ? std::any_cast<rclcpp::Node *>(node)
      ->create_subscription<@(message_typename)>(
        topic_name, 1,
        std::bind(&@(plugin_type_name)::on_new_message, this, _1)) : nullptr}
    , ROOT{topic_name} {}

  mbc::TypedReferences get_references() const final
  {
    return @(',\n      '.join(['{{ROOT, "' + package_name + '/' + type_name + '", mbc::AccessMode::READONLY}'] +
      create_references(message.structure.members, 'mbc::AccessMode::WRITEONLY')))};
  }

  void const * get_value_handle(std::string_view const ref) const
  {
    if (ref == ROOT) {
      return &last_msg_;
@[for descriptor in create_full_type_decriptors(message.structure.members)]@
    } else if (ref == @(to_upper_case('_'.join(descriptor.full_path)))) {
      return &last_msg_.@('.'.join(descriptor.full_path));
@[end for]@
    } else {
      return nullptr;
    }
  }

  void assign_value(std::string_view const, void const *) final {}

  void perform(std::string_view const) final {}

  std::vector<std::string> get_performance_references() const final {return {};}
  
  std::vector<std::string> get_events() const final {
    return {ROOT};
  }

private:
  void on_new_message(const @(message_typename) & msg)
  {
    last_msg_ = msg;
    post_event(ROOT);
  }

  rclcpp::Subscription<@(message_typename)>::SharedPtr subscription_;
  @(message_typename) last_msg_;

  const std::string ROOT;
@[for descriptor in create_full_type_decriptors(message.structure.members)]@
  const std::string @(create_reference_name(descriptor)){ROOT + "/@('/'.join(descriptor.full_path))"};
@[end for]@
};

template class monkey_brain_core::IOPluginFactoryImpl<@(plugin_type_name)>;

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_core::IOPluginFactoryImpl<@(plugin_type_name)>,
  monkey_brain_core::IOPluginFactory)
