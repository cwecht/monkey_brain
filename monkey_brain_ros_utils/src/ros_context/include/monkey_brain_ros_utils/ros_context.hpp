#ifndef MONKEY_BRAIN_ROS_UTILS_ROS_CONTEXT_HPP
#define MONKEY_BRAIN_ROS_UTILS_ROS_CONTEXT_HPP

#include "monkey_brain_core/context.hpp"

#include <rclcpp/node.hpp>

namespace monkey_brain_ros_utils
{

class ROSContext : public monkey_brain_core::Context
{
public:
  explicit ROSContext(rclcpp::Node * node);

  bool is_dry_run() const override {return false;}

  veneer::LoggerPtr get_logger() const override;

private:
  bool holds(const std::type_info & type) const override;

  void * get_concrete_context_impl() const override;

  void const * declare_parameter_impl(
    const std::string & name, void const * value,
    const std::type_info & type) override;

  rclcpp::Node * node_;
};

} // namespace monkey_brain_ros_utils
#endif // MONKEY_BRAIN_ROS_UTILS_ROS_CONTEXT_HPP
