#include "veneer/veneer_stdout_logger.hpp"
#include "veneer/veneer_helper.hpp"

#include <stdio.h>
#include <stdarg.h>

namespace veneer
{

LoggerPtr StdOutLogger::Factory::get_logger(std::string name)
{
  return {new StdOutLogger(std::move(name)), [](Logger * logger) {delete logger;}};
}

StdOutLogger::StdOutLogger(std::string name)
: name_{std::move(name)} {}

bool StdOutLogger::is_enabled_for(LogLevel /*level*/) const {return true;}

void StdOutLogger::log(LogLevel level, const LogLocation & /*log_location*/, const char * fmt, ...)
{
  char * buf = nullptr;
  asprintf(&buf, "%s [%s] %s\n", name_.c_str(), to_string(level), fmt);
  va_list args;
  va_start(args, fmt);
  vfprintf(stdout, buf, args);
  va_end(args);
  free(buf);
}

LoggerPtr StdOutLogger::get_child(const std::string & name)
{
  return {new StdOutLogger(this->name_ + '/' + name), [](Logger * logger) {delete logger;}};
}

} // namespace veneer
