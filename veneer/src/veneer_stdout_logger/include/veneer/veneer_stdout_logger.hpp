#ifndef VENEER_STDOUT_LOGGER__HPP
#define VENEER_STDOUT_LOGGER__HPP

#include "veneer/veneer.hpp"

#include <string>

namespace veneer
{

class StdOutLogger : public Logger {
public:
  class Factory : public LoggerFactory {
public:
    LoggerPtr get_logger(std::string name) override;
  };

  bool is_enabled_for(LogLevel level) const override;
  void log(LogLevel level, const LogLocation & log_location, const char * fmt, ...) override;
  LoggerPtr get_child(const std::string &) override;

private:
  explicit StdOutLogger(std::string name);
  std::string name_;
};

} // namespace veneer
#endif // VENEER_STDOUT_LOGGER__HPP
