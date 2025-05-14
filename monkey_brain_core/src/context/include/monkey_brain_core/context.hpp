#ifndef MONKEY_BRAIN_CORE_CONTEXT_HPP
#define MONKEY_BRAIN_CORE_CONTEXT_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "veneer/veneer.hpp"

namespace monkey_brain_core
{

class Context
{
public:
  virtual ~Context() = default;

  template<typename T>
  T * get_concrete_context() const
  {
    if (not holds(typeid(T))) {
      throw std::runtime_error("Invalid context requested");
    }
    return static_cast<T *>(get_concrete_context_impl());
  }

  template<typename T>
  const T & declare_parameter(const std::string & name, const T & default_value)
  {
    static_assert(is_supported<T>);
    return *static_cast<T const *>(declare_parameter_impl(name, &default_value, typeid(T)));
  }

  template<typename T>
  const T & declare_parameter(const std::string & name)
  {
    static_assert(is_supported<T>);
    return *static_cast<T const *>(declare_parameter_impl(name, nullptr, typeid(T)));
  }

  template<typename T>
  static constexpr bool is_supported = std::is_same_v<T, bool>|| std::is_same_v<T, int64_t>||
    std::is_same_v<T, double>|| std::is_same_v<T, std::string>;

  virtual bool is_dry_run() const = 0;

  virtual veneer::LoggerPtr get_logger() const = 0;

private:
  virtual bool holds(const std::type_info & type) const = 0;
  virtual void * get_concrete_context_impl() const = 0;
  virtual void const * declare_parameter_impl(
    const std::string & name, void const * value,
    const std::type_info & type) = 0;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_CONTEXT_HPP
