#include "monkey_brain_basic_logger/basic_logger.hpp"

#include "monkey_brain_basic_logger/split.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <numeric>

#include <stdexcept>

namespace monkey_brain_basic_logger
{

class PrintableValue
{
public:
  virtual ~PrintableValue() = default;
  virtual std::to_chars_result to_chars(char * first, char * last) const = 0;
  virtual std::size_t expected_length() const = 0;
  virtual void assign_value(const void *) = 0;
};

template<typename T, typename Disambiguator = void>
class Printable;

std::to_chars_result to_chars(const std::string_view value, char * buffer_begin, char * buffer_end)
{
  const auto len = std::min(value.size(),
      static_cast<size_t>(std::distance(buffer_begin, buffer_end)));
  return {std::strncpy(buffer_begin, value.data(), len) + len,
    len != value.size() ? std::errc::value_too_large : std::errc{}};
}

std::to_chars_result to_chars(bool value, char * buffer_begin, char * buffer_end)
{
  const std::string_view str = value ? "true" : "false";
  return to_chars(str, buffer_begin, buffer_end);
}

std::to_chars_result to_chars(const std::string & value, char * buffer_begin, char * buffer_end)
{
  const auto len = std::min(value.size(),
      static_cast<size_t>(std::distance(buffer_begin, buffer_end)));
  return {std::strncpy(buffer_begin, value.c_str(), len) + len,
    len != value.size() ? std::errc::value_too_large : std::errc{}};
}

template<typename T,
  typename = std::enable_if_t<(std::is_integral_v<T> or std::is_floating_point_v<T>) and not std::is_same_v<T, bool>>>
std::to_chars_result to_chars(const T & value, char * buffer_begin, char * buffer_end)
{
  return std::to_chars(buffer_begin, buffer_end, value);
}

std::to_chars_result to_chars(char value, char * buffer_begin, char * buffer_end)
{
  if (buffer_begin != buffer_end) {
    *buffer_begin = value;
    return {buffer_begin + 1, std::errc{}};
  } else {
    return {buffer_end, std::errc::value_too_large};
  }
}

template<typename T>
std::to_chars_result to_chars(const std::vector<T> & vs, char * buffer_begin, char * buffer_end)
{
  auto result = to_chars('[', buffer_begin, buffer_end);
  if (not vs.empty()) {
    result = to_chars(*vs.begin(), result.ptr, buffer_end);

    result = std::accumulate(
        vs.begin() + 1, vs.end(), std::to_chars_result{result.ptr, {}},
      [buffer_end](std::to_chars_result akk, const auto & t) -> std::to_chars_result {
        akk = to_chars(',', akk.ptr, buffer_end);
        akk = to_chars(' ', akk.ptr, buffer_end);
        akk = to_chars(t, akk.ptr, buffer_end);
        return akk;
        });
  }
  return to_chars(']', result.ptr, buffer_end);

}

std::size_t expected_length(bool value)
{
  return value ? std::string_view{"true"}.size() : std::string_view{"false"}.size();
}

std::size_t expected_length(const std::string & value)
{
  return value.size();
}

template<typename T,
  typename = std::enable_if_t<(std::is_integral_v<T> or std::is_floating_point_v<T>)
  and not std::is_same_v<T, bool>>>
std::size_t expected_length(const T & value)
{
  std::array<char, 32> buffer{};
  const auto [ptr, _] = std::to_chars(buffer.begin(), buffer.end(), value);
  return std::distance(buffer.begin(), ptr);
}

template<typename T>
std::size_t expected_length(const std::vector<T> & vs)
{
  if (vs.empty()) {
    return 2UL;
  }
  constexpr auto get_expected_length = [](const T & t) {return expected_length(t) + 2UL;};
  return 2UL + expected_length(*vs.begin()) + std::transform_reduce(
      std::next(vs.begin()), vs.end(), 0UL, std::plus<>{}, get_expected_length);
}

template<typename T>
class Printable<T>: public PrintableValue
{
public:
  Printable() = default;

  std::to_chars_result to_chars(char * buffer_begin, char * buffer_end) const final
  {
    return monkey_brain_basic_logger::to_chars(value_, buffer_begin, buffer_end);
  }

  std::size_t expected_length() const final
  {
    return monkey_brain_basic_logger::expected_length(value_);
  }

  void assign_value(const void * ptr) final
  {
    if (ptr == nullptr) {throw std::runtime_error("can not assign from NULL ptr");}
    value_ = *reinterpret_cast<const T *>(ptr);
  }

private:
  T value_;
};

std::unique_ptr<PrintableValue> create_printable(const monkey_brain_core::ValueType type)
{
  using monkey_brain_core::ValueTypes;
  if (type == ValueTypes::BOOL) {
    return std::make_unique<Printable<bool>>();
  } else if (type == ValueTypes::INT8) {
    return std::make_unique<Printable<std::int8_t>>();
  } else if (type == ValueTypes::UINT8) {
    return std::make_unique<Printable<std::uint8_t>>();
  } else if (type == ValueTypes::INT16) {
    return std::make_unique<Printable<std::int16_t>>();
  } else if (type == ValueTypes::UINT16) {
    return std::make_unique<Printable<std::uint16_t>>();
  } else if (type == ValueTypes::INT32) {
    return std::make_unique<Printable<std::int32_t>>();
  } else if (type == ValueTypes::UINT32) {
    return std::make_unique<Printable<std::uint32_t>>();
  } else if (type == ValueTypes::INT64) {
    return std::make_unique<Printable<std::int64_t>>();
  } else if (type == ValueTypes::UINT64) {
    return std::make_unique<Printable<std::uint64_t>>();
  } else if (type == ValueTypes::FLOAT32) {
    return std::make_unique<Printable<float>>();
  } else if (type == ValueTypes::FLOAT64) {
    return std::make_unique<Printable<double>>();
  } else if (type == ValueTypes::STRING) {
    return std::make_unique<Printable<std::string>>();
  } else if (type == ValueTypes::INT8_ARRAY) {
    return std::make_unique<Printable<std::vector<std::int8_t>>>();
  } else if (type == ValueTypes::UINT8_ARRAY) {
    return std::make_unique<Printable<std::vector<std::uint8_t>>>();
  } else if (type == ValueTypes::INT16_ARRAY) {
    return std::make_unique<Printable<std::vector<std::int16_t>>>();
  } else if (type == ValueTypes::UINT16_ARRAY) {
    return std::make_unique<Printable<std::vector<std::uint16_t>>>();
  } else if (type == ValueTypes::INT32_ARRAY) {
    return std::make_unique<Printable<std::vector<std::int32_t>>>();
  } else if (type == ValueTypes::UINT32_ARRAY) {
    return std::make_unique<Printable<std::vector<std::uint32_t>>>();
  } else if (type == ValueTypes::INT64_ARRAY) {
    return std::make_unique<Printable<std::vector<std::int64_t>>>();
  } else if (type == ValueTypes::UINT64_ARRAY) {
    return std::make_unique<Printable<std::vector<std::uint64_t>>>();
  } else if (type == ValueTypes::FLOAT32_ARRAY) {
    return std::make_unique<Printable<std::vector<float>>>();
  } else if (type == ValueTypes::FLOAT64_ARRAY) {
    return std::make_unique<Printable<std::vector<double>>>();
  } else if (type == ValueTypes::STRING_ARRAY) {
    return std::make_unique<Printable<std::vector<std::string>>>();
  } else {
    throw std::runtime_error("Unsupported type for printing: " + type);
  }
}

BasicLogger::BasicLogger(
  const std::string_view format_string,
  const monkey_brain_core::TypedReferences & references)
{
  const auto ts = split(format_string, "{}");
  tokens_.reserve(ts.size());
  auto ref = references.begin();
  for (auto t : ts) {
    if (t == "{}" && ref != references.end()) {
      auto ptr = tokens_.emplace_back(create_printable(ref->type)).get();
      values_.emplace(ref->reference, ptr);
      ++ref;
    } else if (not t.empty()) {
      std::string str{t};
      tokens_.emplace_back(std::make_unique<Printable<std::string>>())->assign_value(&str);
    }
  }
}

BasicLogger::~BasicLogger() = default;

void
BasicLogger::assign_value(std::string_view const reference, void const * ptr)
{
  const auto value = values_.find(reference);
  if (value == values_.end()) {
    return;
  }
  value->second->assign_value(ptr);
}

std::size_t
BasicLogger::get_expected_length() const
{
  constexpr auto kGetExpectedLength = [](const auto & t) {return t->expected_length();};
  return std::transform_reduce(
    tokens_.begin(),
    tokens_.end(), 0UL, std::plus<>{}, kGetExpectedLength);
}

std::to_chars_result
BasicLogger::to_chars(char * buffer_begin, char * buffer_end) const
{
  return std::accumulate(
    tokens_.begin(), tokens_.end(), std::to_chars_result{buffer_begin, {}},
    [buffer_end](std::to_chars_result akk, const auto & t) -> std::to_chars_result {
      return t->to_chars(akk.ptr, buffer_end);
    });
}

} // namespace monkey_brain_basic_logger
