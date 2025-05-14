#ifndef MONKEY_BRAIN_CORE_VALUE_TYPE_HPP
#define MONKEY_BRAIN_CORE_VALUE_TYPE_HPP

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace monkey_brain_core
{

using ValueType = std::string;

// https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#field-types
struct ValueTypes
{
  inline static const ValueType BOOL{"boolean"};
  inline static const ValueType BOOL_ARRAY{"boolean[]"};
  // BYTE,
  inline static const ValueType CHAR{"char"};
  inline static const ValueType CHAR_ARRAY{"char[]"};
  inline static const ValueType FLOAT32{"float32"};
  inline static const ValueType FLOAT32_ARRAY{"float32[]"};
  inline static const ValueType FLOAT64{"float64"};
  inline static const ValueType FLOAT64_ARRAY{"float64[]"};
  inline static const ValueType INT8{"int8"};
  inline static const ValueType INT8_ARRAY{"int8[]"};
  inline static const ValueType UINT8{"uint8"};
  inline static const ValueType UINT8_ARRAY{"uint8[]"};
  inline static const ValueType INT16{"int16"};
  inline static const ValueType INT16_ARRAY{"int16[]"};
  inline static const ValueType UINT16{"uint16"};
  inline static const ValueType UINT16_ARRAY{"uint16[]"};
  inline static const ValueType INT32{"int32"};
  inline static const ValueType INT32_ARRAY{"int32[]"};
  inline static const ValueType UINT32{"uint32"};
  inline static const ValueType UINT32_ARRAY{"uint32[]"};
  inline static const ValueType INT64{"int64"};
  inline static const ValueType INT64_ARRAY{"int64[]"};
  inline static const ValueType UINT64{"uint64"};
  inline static const ValueType UINT64_ARRAY{"uint64[]"};
  // WSTRING,
  inline static const ValueType STRING{"string"};
  inline static const ValueType STRING_ARRAY{"string[]"};
};

template<typename Rng, typename T>
bool __contains(const Rng & rng, T val)
{
  return std::end(rng) != std::find(std::begin(rng), std::end(rng), val);
}

inline bool is_signed_integer(ValueType v)
{
  const std::string_view signed_integer_types[] = {
    ValueTypes::INT8, ValueTypes::INT16, ValueTypes::INT32, ValueTypes::INT64
  };
  return __contains(signed_integer_types, v);
}

inline bool is_unsigned_integer(ValueType v)
{
  const std::string_view unsigned_integer_types[] = {
    ValueTypes::UINT8, ValueTypes::UINT16, ValueTypes::UINT32, ValueTypes::UINT64
  };
  return __contains(unsigned_integer_types, v);
}

inline bool is_floating_point(ValueType v)
{
  return ValueTypes::FLOAT32 == v || ValueTypes::FLOAT64 == v;
}

template<typename T>
ValueType name()
{
  if constexpr (std::is_same_v<T, std::string>) {
    return ValueTypes::STRING;
  }
  if constexpr (std::is_same_v<T, int64_t>) {
    return ValueTypes::INT64;
  }
  if constexpr (std::is_same_v<T, uint64_t>) {
    return ValueTypes::UINT64;
  }
  if constexpr (std::is_same_v<T, char>) {
    return ValueTypes::CHAR;
  }
  if constexpr (std::is_same_v<T, bool>) {
    return ValueTypes::BOOL;
  }
  return "INVALID";
}

template<typename T>
constexpr bool is_basic_type = std::is_same_v<T, char>||
  std::is_same_v<T, bool>||
  std::is_same_v<T, double>||
  std::is_same_v<T, float>||
  std::is_same_v<T, int8_t>||
  std::is_same_v<T, uint8_t>||
  std::is_same_v<T, int16_t>||
  std::is_same_v<T, uint16_t>||
  std::is_same_v<T, int32_t>||
  std::is_same_v<T, uint32_t>||
  std::is_same_v<T, int64_t>||
  std::is_same_v<T, uint64_t>||
  std::is_same_v<T, std::string>;
enum class AccessMode
{
  READONLY, WRITEONLY, READWRITE
};

struct TypedReference
{
  std::string reference;
  ValueType type;
  AccessMode access_mode;
};

using TypedReferences = std::vector<TypedReference>;

}  // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_VALUE_TYPE_HPP
