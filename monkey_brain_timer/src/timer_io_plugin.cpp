#include "monkey_brain_core/io_plugin_factory.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mbc = monkey_brain_core;

class TimerPlugin : public mbc::IOPlugin
{
public:
  TimerPlugin(
    std::any node, std::string timer_name,
    const YAML::Node & params)
  : timer_{node.has_value() ? std::any_cast<rclcpp::Node *>(node)->create_timer(
        std::chrono::milliseconds {params["period"].as<uint64_t>()},
        std::bind(&TimerPlugin::on_timer, this)) : nullptr}
    , timer_name_{std::move(timer_name)} {}

  mbc::TypedReferences get_references() const final
  {
    return {{timer_name_, mbc::ValueTypes::UINT64, mbc::AccessMode::READONLY}};
  }

  void const * get_value_handle(std::string_view const) const
  {
    return &counter_;
  }

  void assign_value(std::string_view const, void const *) final {}

  void perform(std::string_view const) final {}

  std::vector<std::string> get_performance_references() const final {return {};}

  std::vector<std::string> get_events() const final
  {
    return {timer_name_};
  }

private:
  void on_timer()
  {
    ++counter_;
    post_event(timer_name_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::string timer_name_;
  uint64_t counter_ = 0;
};

template class monkey_brain_core::IOPluginFactoryImpl<TimerPlugin>;

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_core::IOPluginFactoryImpl<TimerPlugin>,
  monkey_brain_core::IOPluginFactory)
