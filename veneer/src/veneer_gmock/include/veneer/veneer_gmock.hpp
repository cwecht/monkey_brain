#ifndef VENEER_GMOCK__HPP
#define VENEER_GMOCK__HPP

#include "veneer/veneer.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdarg.h>

namespace veneer
{

class LoggerMock : public Logger {
public:
  MOCK_CONST_METHOD1(is_enabled_for, bool(LogLevel));
  MOCK_CONST_METHOD4(log_impl,
      void(LogLevel level, const LogLocation & log_location, const char * text,
    const std::string_view));
  void log(LogLevel level, const LogLocation & log_location, const char * fmt, ...) override
  {
    va_list args;
    va_start(args, fmt);
    char * buf = nullptr;
    vasprintf(&buf, fmt, args);
    va_end(args);
    std::unique_ptr<char[], decltype(&::free)> guard{buf, &::free};
    log_impl(level, log_location, fmt, std::string_view{buf});
  }
};

class LoggerFactoryMock : public LoggerFactory {
public:
  MOCK_METHOD1(get_logger, LoggerPtr(std::string));
};

} // namespace veneer
#endif // VENEER_GMOCK__HPP
