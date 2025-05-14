#ifndef MONKEY_BRAIN_CORE_ACTION_HPP
#define MONKEY_BRAIN_CORE_ACTION_HPP

#include <memory>
#include <vector>

namespace monkey_brain_core
{

class Action
{
public:
  virtual ~Action() = default;
  virtual void execute() = 0;
};

using Actions = std::vector<std::unique_ptr<Action>>;

inline void execute(const Actions & actions)
{
  for (const auto & action : actions) {
    action->execute();
  }
}

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_ACTION_HPP
