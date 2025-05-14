#include "monkey_brain_scxml/parse_scxml.hpp"

#include "monkey_brain_core/parse_expression.hpp"
#include "monkey_brain_core/parse_action.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <stdexcept>

#include "tinyxml2.h"

namespace monkey_brain_scxml
{

namespace
{

using namespace tinyxml2;

template<typename Value>
struct NameToParser
{
  std::string_view name;
  Value (* parse)(XMLElement *);
  bool operator==(const char * other) const
  {
    return this->name == other;
  }
};

template<typename Value>
NameToParser(std::string_view, Value (*)(XMLElement *))->NameToParser<Value>;

template<typename Value, std::size_t N>
using NamesToParsers = std::array<NameToParser<Value>, N>;

template<typename Value, std::size_t N>
std::vector<Value> parse_child_elements(
  XMLElement * parent,
  NamesToParsers<Value, N> names_to_parsers)
{
  std::vector<Value> transitions;
  for (XMLElement * child = parent->FirstChildElement(); child != nullptr;
    child = child->NextSiblingElement())
  {
    const auto n2p = std::find(names_to_parsers.begin(), names_to_parsers.end(), child->Name());
    if (n2p != names_to_parsers.end()) {
      const auto parse = n2p->parse;
      transitions.push_back((*parse)(child));
    }
  }
  return transitions;
}

template<typename Value>
std::vector<Value> parse_child_elements(
  XMLElement * parent,
  std::string_view name,
  Value (* parse)(XMLElement *))
{
  const std::array n2ps{NameToParser{name, parse}};
  return parse_child_elements<Value>(parent, n2ps);
}

std::string parse_attribute(char const * const name, XMLElement * state)
{
  const XMLAttribute * id_attribute = state->FindAttribute(name);
  if (id_attribute == nullptr) {
    throw std::invalid_argument("Missing attribute in scxml file: " + std::string{name});
  }

  return id_attribute->Value();
}

std::string parse_optional_attribute(char const * const name, XMLElement * state)
{
  const XMLAttribute * id_attribute = state->FindAttribute(name);
  return (id_attribute == nullptr) ? "" : id_attribute->Value();
}


monkey_brain_core::ActionDescriptions parse_xml_raise_action(XMLElement * action)
{
  return {monkey_brain_core::ActionDescription{
      "RAISE",
      {monkey_brain_core::parse_expression(parse_attribute("event", action)).value()}
    }};
}

monkey_brain_core::ActionDescriptions parse_script(XMLElement * script)
{
  if (script->GetText()) {
    return monkey_brain_core::parse_action(script->GetText());
  } else {
    return {};
  }
}

monkey_brain_core::ActionDescriptions flatten(
  const std::vector<monkey_brain_core::ActionDescriptions> ds)
{
  monkey_brain_core::ActionDescriptions flat_ds;
  for (const auto & actions : ds) {
    flat_ds.insert(flat_ds.end(), actions.begin(), actions.end());
  }
  return flat_ds;
}

monkey_brain_core::ActionDescriptions parse_actions(XMLElement * parent)
{
  const std::array n2ps{NameToParser{"raise", &parse_xml_raise_action},
    NameToParser{"script", &parse_script}};
  return flatten(parse_child_elements(parent, n2ps));
}

Initial parse_initial_tag(XMLElement * initial_tag)
{
  XMLElement * transition = initial_tag->FirstChildElement("transition");
  if (transition == nullptr) {
    throw std::invalid_argument("Initial tag without transition");
  }
  return {parse_attribute("target", transition), parse_actions(transition)};
}

Initial parse_initial(XMLElement * state)
{
  XMLElement * initial_tag = state->FirstChildElement("initial");
  if (initial_tag != nullptr) {
    return parse_initial_tag(initial_tag);
  }

  return {parse_optional_attribute("initial", state), {}};
}

TransitionDescription parse_transition(XMLElement * transition)
{
  return {
    parse_optional_attribute("event", transition),
    monkey_brain_core::parse_expression(parse_optional_attribute("cond", transition)),
    parse_actions(transition),
    parse_optional_attribute("target", transition),
    parse_optional_attribute("type", transition)
  };
}

std::unique_ptr<NestedStates> parse_nested_states(XMLElement * parent);

StateDescription parse_state(XMLElement * state)
{
  return {
    parse_attribute("id", state),
    parse_child_elements(state, "transition", &parse_transition),
    flatten(parse_child_elements(state, "onentry", &parse_actions)),
    flatten(parse_child_elements(state, "onexit", &parse_actions)),
    parse_nested_states(state),
    StateType::REGULAR
  };
}

StateDescription parse_final_state(XMLElement * state)
{
  return {
    parse_attribute("id", state),
    {},
    flatten(parse_child_elements(state, "onentry", &parse_actions)),
    flatten(parse_child_elements(state, "onexit", &parse_actions)),
    {},
    StateType::FINAL
  };
}

StateType parse_history_state_type(XMLElement * state)
{
  const XMLAttribute * type_attribute = state->FindAttribute("type");
  if (type_attribute == nullptr || type_attribute->Value() == std::string_view("shallow")) {
    return StateType::HISTORY_SHALLOW;
  } else if (type_attribute->Value() == std::string_view("deep")) {
    return StateType::HISTORY_DEEP;
  } else {
    throw std::invalid_argument(std::string("Invalid history type: ") + type_attribute->Value());
  }
}

StateDescription parse_history_state(XMLElement * state)
{
  return {
    parse_attribute("id", state),
    parse_child_elements(state, "transition", &parse_transition),
    {}, {}, {}, parse_history_state_type(state)
  };
}

std::vector<StateDescription> parse_states(XMLElement * parent)
{
  const std::array n2ps{
    NameToParser{"state", &parse_state},
    NameToParser{"parallel", &parse_state},
    NameToParser{"final", &parse_final_state},
    NameToParser{"history", &parse_history_state}};
  return parse_child_elements(parent, n2ps);
}

std::unique_ptr<NestedStates> parse_nested_states(XMLElement * parent)
{
  auto states = parse_states(parent);

  if (states.empty()) {
    return nullptr;
  }

  const bool is_parallel = parent->Name() == std::string_view{"parallel"};

  if (is_parallel) {
    return std::unique_ptr<NestedStates>(
      new NestedStates{std::move(states)});
  } else {
    return std::unique_ptr<NestedStates>(
      new NestedStates{std::move(states), parse_initial(parent)});
  }
}

} // namespace

StateMachineDescription parse_scxml(const std::filesystem::path & path_name)
{
  tinyxml2::XMLDocument doc;
  doc.LoadFile(path_name.c_str());

  XMLElement * const root = doc.FirstChildElement("scxml");
  if (root == nullptr) {
    if (doc.ErrorID() == XML_ERROR_FILE_NOT_FOUND ||
      doc.ErrorID() == XML_ERROR_FILE_COULD_NOT_BE_OPENED ||
      doc.ErrorID() == XML_ERROR_FILE_READ_ERROR)
    {
      throw std::invalid_argument("File \"" + path_name.string() + "\" could not be found!");
    } else {
      throw std::invalid_argument(path_name.string() + " is no valid XML. Error at line " +
        std::to_string(doc.ErrorLineNum()) + ": " + doc.ErrorStr());
    }
  }

  return {parse_states(root), parse_initial(root)};
}

} // namespace monkey_brain_scxml
