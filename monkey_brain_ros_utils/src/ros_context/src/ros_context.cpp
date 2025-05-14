#include "monkey_brain_ros_utils/ros_context.hpp"

#include "veneer_ros2/ros2_logger.hpp"

namespace monkey_brain_ros_utils
{
namespace
{

rclcpp::ParameterValue to_rclcpp_parameter_value(void const * value, const std::type_info & type)
{
  using monkey_brain_core::Context;
  if (type == typeid(bool)) {
    static_assert(Context::is_supported<bool>);
    return rclcpp::ParameterValue{*static_cast<bool const *>(value)};
  }
  if (type == typeid(int64_t)) {
    static_assert(Context::is_supported<int64_t>);
    return rclcpp::ParameterValue{*static_cast<int64_t const *>(value)};
  }
  if (type == typeid(double)) {
    static_assert(Context::is_supported<double>);
    return rclcpp::ParameterValue{*static_cast<double const *>(value)};
  }
  if (type == typeid(std::string)) {
    static_assert(Context::is_supported<std::string>);
    return rclcpp::ParameterValue{*static_cast<std::string const *>(value)};
  } else {
    throw std::runtime_error("unsupported type");
  }
}

void const * from_rclcpp_parameter_value(
  const rclcpp::ParameterValue & value,
  const std::type_info & type)
{
  using monkey_brain_core::Context;
  if (type == typeid(bool)) {
    static_assert(Context::is_supported<bool>);
    return &value.get<bool>();
  }
  if (type == typeid(int64_t)) {
    static_assert(Context::is_supported<int64_t>);
    return &value.get<int64_t>();
  }
  if (type == typeid(double)) {
    static_assert(Context::is_supported<double>);
    return &value.get<double>();
  }
  if (type == typeid(std::string)) {
    static_assert(Context::is_supported<std::string>);
    return &value.get<std::string>();
  } else {
    throw std::runtime_error("unsupported type");
  }
}

rclcpp::ParameterType to_rclcpp_type(const std::type_info & type)
{
  using monkey_brain_core::Context;
  if (type == typeid(bool)) {
    static_assert(Context::is_supported<bool>);
    return rclcpp::ParameterType::PARAMETER_BOOL;
  }
  if (type == typeid(int64_t)) {
    static_assert(Context::is_supported<int64_t>);
    return rclcpp::ParameterType::PARAMETER_INTEGER;
  }
  if (type == typeid(double)) {
    static_assert(Context::is_supported<double>);
    return rclcpp::ParameterType::PARAMETER_DOUBLE;
  }
  if (type == typeid(std::string)) {
    static_assert(Context::is_supported<std::string>);
    return rclcpp::ParameterType::PARAMETER_STRING;
  } else {
    throw std::runtime_error("unsupported type");
  }
}

} // namespace

ROSContext::ROSContext(rclcpp::Node * node)
: node_{node} {}

veneer::LoggerPtr ROSContext::get_logger() const
{
  return veneer_ros2::ROS2Logger::make(node_);
}

bool ROSContext::holds(const std::type_info & type) const
{
  return type == typeid(rclcpp::Node);
}

void * ROSContext::get_concrete_context_impl() const
{
  return node_;
}

void const * ROSContext::declare_parameter_impl(
  const std::string & name, void const * value,
  const std::type_info & type)
{
  if (value == nullptr) {
    return from_rclcpp_parameter_value(
      node_->declare_parameter(
        name, to_rclcpp_type(type),
        rcl_interfaces::msg::ParameterDescriptor(), false), type);

  } else {
    return from_rclcpp_parameter_value(
      node_->declare_parameter(
        name, to_rclcpp_parameter_value(value, type),
        rcl_interfaces::msg::ParameterDescriptor(), false), type);
  }
}

} // namespace monkey_brain_ros_utils
