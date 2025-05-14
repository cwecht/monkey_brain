#ifndef MONKEY_BRAIN_CORE_DECISION_ENGINE_HPP
#define MONKEY_BRAIN_CORE_DECISION_ENGINE_HPP

#include "monkey_brain_core/context.hpp"
#include "monkey_brain_core/environment.hpp"
#include "monkey_brain_core/event_recipient.hpp"
#include "monkey_brain_core/operations_factory.hpp"

#include <functional>

namespace monkey_brain_core
{

class DecisionEngine : public monkey_brain_core::EventRecipient,
  public monkey_brain_core::InternalEventRecipient
{
public:
  DecisionEngine() = default;
  ~DecisionEngine() = default;
  DecisionEngine(const DecisionEngine &) = delete;
  DecisionEngine & operator=(const DecisionEngine &) & = delete;
  DecisionEngine(DecisionEngine &&) = delete;
  DecisionEngine & operator=(DecisionEngine &&) & = delete;
};

class DecisionEngineFactory
{
public:
  virtual ~DecisionEngineFactory() = default;
  virtual std::unique_ptr<DecisionEngine>
  create(
    monkey_brain_core::Context * context, std::function<void()> on_shutdown,
    monkey_brain_core::OperationsFactory & ops_factory,
    monkey_brain_core::Environment & env) const = 0;
};

} // namespace monkey_brain_core
#endif // MONKEY_BRAIN_CORE_DECISION_ENGINE_HPP
