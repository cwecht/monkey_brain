#ifndef VENEER_ROS2_LOGGER__HPP
#define VENEER_ROS2_LOGGER__HPP

#include "veneer/veneer.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <string>

namespace veneer_ros2
{

class ROS2Logger : public veneer::Logger {
public:
  class Factory : public veneer::LoggerFactory {
public:
    veneer::LoggerPtr get_logger(std::string name) override;
  };

  bool is_enabled_for(veneer::LogLevel level) const override;
  void log(
    veneer::LogLevel level, const veneer::LogLocation & log_location, const char * fmt,
    ...) override;
  veneer::LoggerPtr get_child(const std::string &) override;

  static veneer::LoggerPtr make(const rclcpp::Node * node);

private:
  explicit ROS2Logger(const std::string & name);
  explicit ROS2Logger(const rclcpp::Node * node);
  ROS2Logger(rclcpp::Logger & parent, const std::string & name);
  rclcpp::Logger logger_;
};

} // namespace veneer_ros2
#endif // VENEER_ROS2_LOGGER__HPP
