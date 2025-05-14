#ifndef MONKEY_BRAIN_CORE_EXPRESSION_HPP
#define MONKEY_BRAIN_CORE_EXPRESSION_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace monkey_brain_core
{

class ExpressionBase
{
public:
  virtual ~ExpressionBase() = default;
};

using ExpressionPtr = std::unique_ptr<ExpressionBase>;
using Expressions = std::vector<ExpressionPtr>;

template<typename T>
inline constexpr std::size_t size_of = sizeof(T);

template<>
inline constexpr std::size_t size_of<void> = 0UL;

template<typename T>
inline constexpr bool is_check_to_return_by_value =
  std::is_trivially_copyable_v<T>&& (size_of<T>< sizeof(std::int64_t) * 2);

template<typename T>
using ExpressionReturnType = std::conditional_t<is_check_to_return_by_value<T>, T, T const &>;

template<typename ExpressionType>
class Expression : public ExpressionBase
{
public:
  using ReturnType = ExpressionReturnType<ExpressionType>;
  virtual ReturnType get() const = 0;

  template<typename T = ExpressionType,
    typename = typename std::enable_if_t<std::is_same_v<T, bool>>>
  bool holds()
  {
    return get();
  }
};

template<>
class Expression<void>: public ExpressionBase
{
public:
  virtual void const * get() const = 0;
};

struct ExpressionWithType
{
  ExpressionPtr expression;
  std::string type;
};

template<typename ValueType, bool is_cheap_to_copy = is_check_to_return_by_value<ValueType>>
struct ValueStore;

template<typename ValueType>
struct ValueStore<ValueType, true>
{
  ValueType store(ValueType v) const {return v;}
};

template<typename ValueType>
struct ValueStore<ValueType, false>
{
  mutable ValueType m_value;
  ValueType const & store(ValueType v) const
  {
    m_value = std::move(v);
    return m_value;
  }
};

template<typename ValueType>
class FunctionExpression : public Expression<ValueType>, protected ValueStore<ValueType> {};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_EXPRESSION_HPP
