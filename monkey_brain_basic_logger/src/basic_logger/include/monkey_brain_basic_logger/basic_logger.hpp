#ifndef MONKEY_BRAIN_BASIC_LOGGER_BASIC_LOGGER_HPP
#define MONKEY_BRAIN_BASIC_LOGGER_BASIC_LOGGER_HPP

#include "monkey_brain_core/value_types.hpp"

#include <charconv>
#include <memory>
#include <unordered_map>

namespace monkey_brain_basic_logger
{

class PrintableValue;

class BasicLogger
{
public:
  BasicLogger(
    const std::string_view format_string,
    const monkey_brain_core::TypedReferences & references);
  ~BasicLogger();

  void assign_value(std::string_view const reference, void const * ptr);

  std::size_t get_expected_length() const;

  std::to_chars_result to_chars(char * buffer_begin, char * buffer_end) const;

private:
  std::vector<std::unique_ptr<PrintableValue>> tokens_;
  std::unordered_map<std::string_view, PrintableValue *> values_;
};

} // namespace monkey_brain_basic_logger
#endif // MONKEY_BRAIN_BASIC_LOGGER_BASIC_LOGGER_HPP
