#include "monkey_brain_core/cli_context.hpp"

#include "veneer/veneer_stdout_logger.hpp"

#include <algorithm>
#include <sstream>

namespace monkey_brain_core
{
namespace
{

void * parse_parameter(const std::type_info & type, const std::string & param)
{
  std::stringstream ss{param};
  using monkey_brain_core::Context;
  if (type == typeid(bool)) {
    static_assert(Context::is_supported<bool>);
    static bool boolean;
    ss >> std::boolalpha >> boolean;
    return &boolean;
  }
  if (type == typeid(int64_t)) {
    static_assert(Context::is_supported<int64_t>);
    static int64_t integer;
    ss >> integer;
    return &integer;
  }
  if (type == typeid(double)) {
    static_assert(Context::is_supported<double>);
    static double floating;
    ss >> floating;
    return &floating;
  }
  if (type == typeid(std::string)) {
    static_assert(Context::is_supported<std::string>);
    static std::string para;
    para = param;
    return &para;
  } else {
    throw std::runtime_error("unsupported type");
  }
  return nullptr;
}

} // namespace

CLIContext::CLIContext(int argc, const char * const argv[])
: argc_{argc}, argv_{argv} {}

void const * CLIContext::declare_parameter_impl(
  const std::string & name, void const * value,
  const std::type_info & type)
{
  const auto parameter_prefix = name + ":=";
  const auto starts_with = [&parameter_prefix] (std::string_view arg) {
      return arg.find(parameter_prefix) == 0;
    };
  const auto argv_end = argv_ + argc_;
  auto p = std::find_if(argv_, argv_end, starts_with);
  if (p == argv_end) {
    if (value != nullptr) {
      return value;
    } else {
      throw std::runtime_error("parameter '" + name + "' could not be found");
    }
  }

  const std::string param = *p + parameter_prefix.size();
  return parse_parameter(type, param);
}

veneer::LoggerPtr CLIContext::get_logger() const
{
  return veneer::StdOutLogger::Factory().get_logger("");
}

} // namespace monkey_brain_core
