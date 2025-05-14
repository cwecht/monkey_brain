#ifndef MONKEY_BRAIN_CORE_DYNAMIC_CAST_MOVE_HPP
#define MONKEY_BRAIN_CORE_DYNAMIC_CAST_MOVE_HPP

#include <memory>

namespace monkey_brain_core
{

template<typename Target, typename Source>
std::unique_ptr<Target> dynamic_cast_move(Source && s)
{
  return std::unique_ptr<Target>(dynamic_cast<Target *>(s.release()));
}

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_DYNAMIC_CAST_MOVE_HPP
