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

protected:
  ExpressionBase() = default;
  ExpressionBase(const ExpressionBase &) = delete;
  ExpressionBase(ExpressionBase &&) = delete;
  ExpressionBase & operator=(const ExpressionBase &) & = delete;
  ExpressionBase & operator=(ExpressionBase &&) & = delete;
};

using ExpressionPtr = std::unique_ptr<ExpressionBase>;
using Expressions = std::vector<ExpressionPtr>;

template<typename T>
inline constexpr std::size_t SIZE_OF = sizeof(T);

template<>
inline constexpr std::size_t SIZE_OF<void> = 0UL;

template<typename T>
inline constexpr bool IS_CHEAP_TO_RETURN_BY_VALUE =
  std::is_trivially_copyable_v<T>&& (SIZE_OF<T>< sizeof(std::int64_t) * 2);

template<typename T>
using ExpressionReturnType = std::conditional_t<IS_CHEAP_TO_RETURN_BY_VALUE<T>, T, T const &>;

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

template<typename ValueType, bool is_cheap_to_copy = IS_CHEAP_TO_RETURN_BY_VALUE<ValueType>>
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
