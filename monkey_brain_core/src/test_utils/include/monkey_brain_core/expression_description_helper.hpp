#ifndef MONKEY_BRAIN_CORE_EXPRESSION_DESCRIPTION_HELPER_HPP
#define MONKEY_BRAIN_CORE_EXPRESSION_DESCRIPTION_HELPER_HPP

#include "monkey_brain_core/expression_description.hpp"

monkey_brain_core::ExpressionDescription REF(std::string value)
{
  return monkey_brain_core::ExpressionDescription{"REFERENCE", {}, {value}};
}

monkey_brain_core::ExpressionDescription VALUE(int64_t value)
{
  return monkey_brain_core::ExpressionDescription{"VALUE", {}, {value}};
}

monkey_brain_core::ExpressionDescription VALUE(uint64_t value)
{
  return monkey_brain_core::ExpressionDescription{"VALUE", {}, {value}};
}

monkey_brain_core::ExpressionDescription VALUE(double value)
{
  return monkey_brain_core::ExpressionDescription{"VALUE", {}, {value}};
}

template<typename T, typename = std::enable_if_t<std::is_same_v<T, bool>>>
monkey_brain_core::ExpressionDescription VALUE(T value)
{
  return monkey_brain_core::ExpressionDescription{"VALUE", {}, {value}};
}

monkey_brain_core::ExpressionDescription STR(std::string value)
{
  return monkey_brain_core::ExpressionDescription{"VALUE", {}, value};
}

monkey_brain_core::ExpressionDescription NOT(monkey_brain_core::ExpressionDescription sub)
{
  return monkey_brain_core::ExpressionDescription{"NOT", {std::move(sub)}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription AND(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"AND", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription OR(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"OR", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription MODULO(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"%", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription EQUALS(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"==", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription NEQ(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"!=", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription PLUS(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"+", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription FUN(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"fun", {std::move(sub)...}, {}};
}

template<typename ... Ts>
monkey_brain_core::ExpressionDescription GUN(Ts... sub)
{
  return monkey_brain_core::ExpressionDescription{"gun", {std::move(sub)...}, {}};
}

monkey_brain_core::ExpressionDescription UNSIGNED(monkey_brain_core::ExpressionDescription sub)
{
  return monkey_brain_core::ExpressionDescription{"UNSIGNED", {std::move(sub)}, {}};
}

#endif // MONKEY_BRAIN_CORE_EXPRESSION_DESCRIPTION_HELPER_HPP
