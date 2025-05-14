#include "monkey_brain_core/io_plugin_factory.hpp"

#include "monkey_brain_basic_logger/basic_logger.hpp"


#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mbc = monkey_brain_core;

namespace monkey_brain_basic_logger
{

class BasicLoggerPlugin : public mbc::IOPlugin
{
public:
  BasicLoggerPlugin(
    std::any node, std::string name,
    const YAML::Node & params)
  : name_{std::move(name)}
    , logger_{node.has_value() ? std::any_cast<rclcpp::Node *>(node)->get_logger() :
      rclcpp::get_logger("dummy")}
    , format_string_{node.has_value() ? params["format"].as<std::string>() : ""}
    , severity_{node.has_value() ? params["severity"].as<int>() : 0}
    , references_{node.has_value() ? read_references(params["params"],
        name_) : mbc::TypedReferences{}}
    , basic_logger_{format_string_, references_}
  {
  }

  mbc::TypedReferences get_references() const final
  {
    return references_;
  }

  void const * get_value_handle(std::string_view const) const
  {
    return nullptr;
  }

  void assign_value(std::string_view const reference, void const * ptr) final
  {
    basic_logger_.assign_value(reference, ptr);
  }

  void perform(std::string_view const) final
  {
    const std::size_t size = basic_logger_.get_expected_length();

    std::string buffer(size, '\0');
    const auto result = basic_logger_.to_chars(buffer.data(), buffer.data() + size + 1UL);
    buffer.resize(std::distance(buffer.data(), result.ptr));
    static rcutils_log_location_t __rcutils_logging_location = {__func__, __FILE__, __LINE__};
    if (rcutils_logging_logger_is_enabled_for(logger_.get_name(), severity_)) {
      rcutils_log_internal(
        &__rcutils_logging_location, severity_,
        logger_.get_name(), "%s", buffer.c_str());
    }
  }

  std::vector<std::string> get_performance_references() const final
  {
    return {name_ + "/print"};
  }

  std::vector<std::string> get_events() const final
  {
    return {};
  }

private:
  static mbc::TypedReferences read_references(const YAML::Node & map, const std::string & prefix)
  {
    mbc::TypedReferences references;
    for (const auto & e : map) {
      references.push_back(
        mbc::TypedReference{
          prefix + '/' + e.first.as<std::string>(),
          e.second.as<std::string>(),
          mbc::AccessMode::WRITEONLY});
    }
    return references;
  }
  std::string name_;
  rclcpp::Logger logger_;
  std::string format_string_;
  int severity_;
  mbc::TypedReferences references_;
  BasicLogger basic_logger_;
};

} // namespace monkey_brain_basic_logger

using namespace monkey_brain_basic_logger;
template class monkey_brain_core::IOPluginFactoryImpl<BasicLoggerPlugin>;

PLUGINLIB_EXPORT_CLASS(
  monkey_brain_core::IOPluginFactoryImpl<BasicLoggerPlugin>,
  monkey_brain_core::IOPluginFactory)
