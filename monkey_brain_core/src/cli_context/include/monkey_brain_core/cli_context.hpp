#ifndef MONKEY_BRAIN_CORE_CLI_CONTEXT_HPP
#define MONKEY_BRAIN_CORE_CLI_CONTEXT_HPP

#include "monkey_brain_core/context.hpp"

#include <memory>
#include <stdexcept>
#include <typeinfo>

namespace monkey_brain_core
{

class CLIContext : public Context {
public:
  CLIContext(int argc, const char * const argv[]);

  bool is_dry_run() const override {return true;}

  veneer::LoggerPtr get_logger() const override;

private:
  bool holds(const std::type_info &) const override {return false;}
  void * get_concrete_context_impl() const override {return nullptr;}

  void const * declare_parameter_impl(
    const std::string & name, void const * value,
    const std::type_info & type) override;

  int argc_;
  const char * const * argv_;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_CLI_CONTEXT_HPP
