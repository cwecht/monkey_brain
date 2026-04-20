#ifndef VENEER_HELPER_HPP
#define VENEER_HELPER_HPP

#include "veneer/veneer.hpp"

namespace veneer
{

constexpr inline const char * to_string(LogLevel level)
{
  switch(level) {
    case LOG_DEBUG: return "Debug";
    case LOG_INFO: return "Info";
    case LOG_WARN: return "Warning";
    case LOG_ERROR: return "Error";
    default: return "None";
  }
}

} // namespace veneer
#endif // VENEER_HELPER_HPP
