#ifndef MONKEY_BRAIN_SCXML_SCXML_STATE_MACHINE_FACTORY_HPP
#define MONKEY_BRAIN_SCXML_SCXML_STATE_MACHINE_FACTORY_HPP

#include "monkey_brain_core/decision_engine.hpp"

namespace monkey_brain_scxml
{

class SCXMLStateMachineFactory : public monkey_brain_core::DecisionEngineFactory
{
public:
  std::unique_ptr<monkey_brain_core::DecisionEngine>
  create(
    monkey_brain_core::Context * context,
    std::function<void()> on_shutdown,
    monkey_brain_core::OperationsFactory & ops_factory,
    monkey_brain_core::Environment & env) const override;
};

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_SCXML_SCXML_STATE_MACHINE_FACTORY_HPP
