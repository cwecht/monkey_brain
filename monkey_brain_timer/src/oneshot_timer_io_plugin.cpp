#include "monkey_brain_core/io_plugin_factory.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mbc = monkey_brain_core;

class OneshotTimerPlugin : public mbc::IOPlugin
{
public:
  OneshotTimerPlugin(
    std::any node, std::string timer_name,
    const YAML::Node & params)
  : timer_{node.has_value() ? std::any_cast<rclcpp::Node *>(node)->create_timer(
        std::chrono::milliseconds {params["timeout"].as<uint64_t>()},
        std::bind(&OneshotTimerPlugin::on_timer, this)) : nullptr}
    , timer_name_{std::move(timer_name)}
  {
    if (timer_) {
      timer_->cancel();
    }
  }

  mbc::TypedReferences get_references() const final
  {
    return {};
  }

  void const * get_value_handle(std::string_view const) const final
  {
    return nullptr;
  }

  void assign_value(std::string_view const, void const *) final {}

  void perform(std::string_view const) override
  {
    timer_->reset();
  }

  std::vector<std::string> get_performance_references() const final
  {
    return {timer_name_ + "/reset_timer"};
  }

  std::vector<std::string> get_events() const final
  {
    return {timer_name_};
  }

private:
  void on_timer()
  {
    timer_->cancel();
    post_event(timer_name_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::string timer_name_;
};

template class monkey_brain_core::IOPluginFactoryImpl<OneshotTimerPlugin>;

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_core::IOPluginFactoryImpl<OneshotTimerPlugin>,
  monkey_brain_core::IOPluginFactory)
