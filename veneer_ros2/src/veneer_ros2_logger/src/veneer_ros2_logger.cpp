#include "veneer_ros2/ros2_logger.hpp"
#include "veneer/veneer_helper.hpp"

#include <stdio.h>
#include <stdarg.h>

namespace veneer_ros2
{

namespace
{
int to_rcutils_severitiy(veneer::LogLevel level)
{
  using veneer::LogLevel;
  switch(level) {
    case LogLevel::LOG_DEBUG: return RCUTILS_LOG_SEVERITY_DEBUG;
    case LogLevel::LOG_INFO: return RCUTILS_LOG_SEVERITY_INFO;
    case LogLevel::LOG_WARN: return RCUTILS_LOG_SEVERITY_WARN;
    case LogLevel::LOG_ERROR: return RCUTILS_LOG_SEVERITY_ERROR;
    default: return RCUTILS_LOG_SEVERITY_DEBUG;
  }
}
} // namespace

veneer::LoggerPtr ROS2Logger::Factory::get_logger(std::string name)
{
  return {new ROS2Logger(name), [](Logger * logger) {delete logger;}};
}

ROS2Logger::ROS2Logger(const rclcpp::Node * node)
: logger_{node->get_logger()} {}

ROS2Logger::ROS2Logger(const std::string & name)
: logger_{rclcpp::get_logger(name)} {}

ROS2Logger::ROS2Logger(rclcpp::Logger & parent, const std::string & name)
: logger_{parent.get_child(name)} {}

bool ROS2Logger::is_enabled_for(veneer::LogLevel level) const
{
  return rcutils_logging_logger_is_enabled_for(logger_.get_name(), to_rcutils_severitiy(level));
}

void ROS2Logger::log(
  veneer::LogLevel level, const veneer::LogLocation & log_location,
  const char * fmt, ...)
{
  rcutils_log_location_t location = {log_location.func, log_location.file, log_location.line};
  va_list args;
  va_start(args, fmt);
  char * text = nullptr;
  vasprintf(&text, fmt, args);
  va_end(args);
  rcutils_log(&location, to_rcutils_severitiy(level), logger_.get_name(), "%s", text);
  free(text);
}

veneer::LoggerPtr ROS2Logger::get_child(const std::string & name)
{
  return {new ROS2Logger(logger_, name), [](Logger * logger) {delete logger;}};
}

veneer::LoggerPtr ROS2Logger::make(const rclcpp::Node * node)
{
  return {new ROS2Logger(node), [](Logger * logger) {delete logger;}};
}
} // namespace veneer_ros2
