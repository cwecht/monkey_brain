#ifndef VENEER__HPP
#define VENEER__HPP

#include <cstdint>
#include <memory>
#include <string>

namespace veneer
{

enum LogLevel : int8_t
{
  LOG_DEBUG = 0,
  LOG_INFO,
  LOG_WARN,
  LOG_ERROR
};

struct LogLocation
{
  const char * func;
  const char * file;
  size_t line;
};

class Logger;
using LoggerDeleter = void(*)(Logger *);
using LoggerPtr = std::unique_ptr<Logger, LoggerDeleter>;

class Logger {
public:
  virtual ~Logger() = default;
  virtual bool is_enabled_for(LogLevel level) const = 0;
  virtual void log(LogLevel level, const LogLocation & log_location, const char * text, ...) = 0;
  virtual LoggerPtr get_child(const std::string & name) = 0;
};

class LoggerFactory {
public:
  virtual ~LoggerFactory() = default;
  virtual LoggerPtr get_logger(std::string name) = 0;
};

class ZeroLogger : public Logger {
public:
  class Factory : public LoggerFactory {
public:
    LoggerPtr get_logger(std::string /*name*/) override
    {
      return get_instance();
    }
  };

  bool is_enabled_for(LogLevel) const final {return false;}
  void log(LogLevel, const LogLocation &, const char *, ...) final {}
  LoggerPtr get_child(const std::string &) final
  {
    return ZeroLogger::get_instance();
  }

  static LoggerPtr get_instance()
  {
    static ZeroLogger instance;
    return LoggerPtr{&instance, [] (Logger *){}};
  }

private:
  explicit ZeroLogger() = default;
};

#define VENEER_LOG(level, logger, ...) \
  if (logger != nullptr && logger->is_enabled_for(level)) { \
    constexpr veneer::LogLocation log_location{__func__, __FILE__, __LINE__}; \
    logger->log((level), log_location, __VA_ARGS__); \
  }

#define VENEEER_LOG_SEVERITY_DEBUG 0
#define VENEEER_LOG_SEVERITY_INFO 1
#define VENEEER_LOG_SEVERITY_WARN 2
#define VENEEER_LOG_SEVERITY_ERROR 3

#ifndef VENEER_LOG_MIN_SEVERITY
#define VENEER_LOG_MIN_SEVERITY VENEEER_LOG_SEVERITY_DEBUG
#endif

#if (VENEER_LOG_MIN_SEVERITY > VENEEER_LOG_SEVERITY_DEBUG)
  #define VENEER_LOG_DEBUG(...)
#else
  #define VENEER_LOG_DEBUG(...) VENEER_LOG(veneer::LOG_DEBUG, __VA_ARGS__)
#endif

#if (VENEER_LOG_MIN_SEVERITY > VENEEER_LOG_SEVERITY_INFO)
  #define VENEER_LOG_INFO(...)
#else
  #define VENEER_LOG_INFO(...) VENEER_LOG(veneer::LOG_INFO, __VA_ARGS__)
#endif

#if (VENEER_LOG_MIN_SEVERITY > VENEEER_LOG_SEVERITY_WARN)
  #define VENEER_LOG_WARN(...)
#else
  #define VENEER_LOG_WARN(...) VENEER_LOG(veneer::LOG_WARN, __VA_ARGS__)
#endif

#if (VENEER_LOG_MIN_SEVERITY > VENEEER_LOG_SEVERITY_ERROR)
  #define VENEER_LOG_ERROR(...)
#else
  #define VENEER_LOG_ERROR(...) VENEER_LOG(veneer::LOG_ERROR, __VA_ARGS__)
#endif

} // namespace veneer
#endif // VENEER__HPP
