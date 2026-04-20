#ifndef MONKEY_BRAIN_CORE_EXPRESSION_DESCRIPTION_HPP
#define MONKEY_BRAIN_CORE_EXPRESSION_DESCRIPTION_HPP

#include <cstdint>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace monkey_brain_core
{

struct ExpressionDescription
{
  std::string type;
  std::vector<ExpressionDescription> sub_expressions{};
  std::variant<std::monostate, std::string, uint64_t, int64_t, double, char, bool> arg;

private:
  decltype(auto) memberwise() const {return std::tie(type, sub_expressions, arg);}

  friend bool operator==(const ExpressionDescription & a, const ExpressionDescription & b)
  {
    return a.memberwise() == b.memberwise();
  }
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_EXPRESSION_DESCRIPTION_HPP
