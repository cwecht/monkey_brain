#ifndef MONKEY_BRAIN_CORE_CONDITION_HPP
#define MONKEY_BRAIN_CORE_CONDITION_HPP

#include "monkey_brain_core/expression.hpp"

#include <memory>
#include <vector>

namespace monkey_brain_core
{

using Condition = Expression<bool>;
using ConditionPtr = std::unique_ptr<Condition>;
using Conditions = std::vector<ConditionPtr>;

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_CONDITION_HPP
