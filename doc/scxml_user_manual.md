# Users Manual

`monkey_brain_scxml` does not implement the [SCXML specification](https://www.w3.org/TR/scxml/) entirely.
This document is meant to document the supported elements.

## Current Status
All the following tags from the scxml standard are supported: `<scxml>`, `<state>`, `<parallel>`, `<transition>`, `<initial>`, `<final>`, `<history>`, `<onentry>`, `<onexit>`, `<raise>`, `<script>`. For some tags, not all attributes may be supported. See below for further description.
  
### State Machine (`<scxml>`)
The top-level wrapper element, which carries version information. The actual state machine consists of its children. Note that only one of the children is active at any one time.

#### Attributes
##### `initial`
The id of the initial state(s) for the scxml document. If not specified, the default initial state is the first child state in document order.

If multiple states are specified, any two of them must have a `<parallel>` state as their nearest common ancestor and they must not be descendants of each other.

If not all children of a `<parallel>` state is included, the remaining children will enter their initial state.

##### `xmlns`
This attribute is required and must be `"http://www.w3.org/2005/07/scxml"`.

##### `version`
This attribute is required and must be `"1.0"`

#### Unsupported Attributes

`datamodel` and `binding` are not supported as they don't make much sense within a monkey brain context.
The `name` attribute is ignored.

#### Valid Children

- [`<state>`](scxml_user_manual.md#state-state)
- [`<parallel>`](scxml_user_manual.md#parallel-state-parallel)
- [`<final>`](scxml_user_manual.md#final-state-final)
- [`<history>`](scxml_user_manual.md#history-state-final)

#### Example
```
<scxml initial="hello" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
 ...
</scxml>
```

### State (`<state>`)
This element represents a state in the state machine. A state can hold child states to form a hierarchical state machine.

A state may specify either an `initial` attribute or an `<initial>` child element, but not both (`<initial>` takes
prescedence over `initial`).
If none of them are present, the state's first child state in document order is used as initial state.

If the state has any child states, any transition which takes this state as its target will result in the state machine also taking the transition to its initial state. 

#### Attributes

##### `id`
An identifier for the state. This attribute is required.

##### `initial`
The id of the default initial state (or states) for this state. Alternatively, the initial state can be set with the `<initial>` child.

The initial state(s) specified must be descendant(s) of the containing `<state>`.

If multiple states are specified, any two of them must have a `<parallel>` state as their nearest common ancestor and they must not be descendants of each other.

If not all children of a `<parallel>` state is included, the remaining children will enter their initial state.

#### Valid Children
- [`<onentry>`](scxml_user_manual.md#enter-action-onentry)
- [`<onexit>`](scxml_user_manual.md#exit-action-onexit)
- [`<transition>`](scxml_user_manual.md#transition-transition)
- [`<initial>`](scxml_user_manual.md#initial-initial)
- [`<state>`](scxml_user_manual.md#state-state)
- [`<parallel>`](scxml_user_manual.md#parallel-state-parallel)
- [`<final>`](scxml_user_manual.md#final-state-final)
- [`<history>`](scxml_user_manual.md#history-state-final)

#### Example
```
<state id="active" initial="collect_coins">
  ...
</state>
```

### Parallel State (`<parallel>`)

The `<parallel>` element encapsulates a set of child states which are simultaneously active when the parent element is active.

#### Attributes

##### `id`
An identifier for the state. This attribute is required.

#### Valid Children
- [`<onentry>`](scxml_user_manual.md#enter-action-onentry)
- [`<onexit>`](scxml_user_manual.md#exit-action-onexit)
- [`<transition>`](scxml_user_manual.md#transition-transition)
- [`<state>`](scxml_user_manual.md#state-state)
- [`<parallel>`](scxml_user_manual.md#parallel-state-parallel)
- [`<final>`](scxml_user_manual.md#final-state-final)
- [`<history>`](scxml_user_manual.md#history-state-final)

#### Example
```
<parallel id="s11p1">
  ...
</parallel>

```
### Transition (`<transition>`)
This element specifies transitions between states.
The transitions are triggered by events.
They may contain conditions and executable content.

The source state of the transition is the state in which the transition is a child of.

When a transition is executed, the current state and its parents are exited up to the nearest common ancestor of the current state and the transition target.
Then, all states are entered from the ancestor to the transition's target.

If a parallel state is exited, the other parallel state's children are exited also.

At least one of the attributes `event`, `cond` or `target` must be specified.

If multiple transitions may be triggered by an event, one of these transitions are proiritized.
If T1 and T2 are two conflicting transitions and their condition evaluates to true, T1 is prioritized if T1's source state is a descendant of T2's source state.
Otherwise they are prioritized in document order.

#### Attributes

##### `event`
A space separated list of events that trigger this transition.

##### `cond`
An execute condition (see [Monkey Brain Core Concepts]((core_concepts.md#conditions) for details).
The transition is only executed if the condition evaluates to true.

##### `target`
The id of the state to transition to.

##### `type`
Can be `"internal"` or `"external"`. If omitted, the type is external.
If the type is internal and the target state is a descendant of the transitions source state, the transition will not exit and re-enter its source state, while an external one will.
Internal transitions are useful for initial transitions as a child of [`<initial>`](scxml_user_manual.md#initial-initial)

#### Valid Children
- [`<raise>`](scxml_user_manual.md#raise-raise)
- [`<script>`](scxml_user_manual.md#script-script)

#### Example
```
<transition event="cancel" target="coin_return">
  ...
</transition>
```

### Initial (`<initial>`)
This element represents the default initial state for a <state> element.

Note that the child transition must not contain `cond` or `event` attributes, and must specify a `target` whose value is state(s) consisting solely of descendants of the containing state.

#### Attributes
None.

#### Valid Children
- [`<transition>`](scxml_user_manual.md#transition-transition)

#### Example
```
<initial>
  ...
</initial>
```

### Final State (`<final>`)
`<final>` represents a final state of the state machine.

When a final state is entered, a `done.state.ID` event is raised, where ID is the id of the parent state. If this parent state is a child of a `<parallel>` element, and all of the `<parallel>`'s other children are also in final states, a `done.state.ID` event is raised where ID is the id of the `<parallel>` element.

If the entered final state is a child of the `<scxml>` element, the state machine must terminate. This is currently not implemented.

#### Attributes

##### `id`
An identifier for the state. This attribute is optional.

#### Valid Children
- [`<onentry>`](scxml_user_manual.md#enter-action-onentry)
- [`<onexit>`](scxml_user_manual.md#exit-action-onexit)

#### Example
```
<final id="dispense_coke">
  ...
</final>
```

### History State (`<history>`)
The `<history>` pseudo-state allows a state machine to remember its state configuration.
A `<transition>` taking the `<history>` state as its target will return the state machine to this recorded configuration.

#### Attributes

##### `id`
An identifier for the state. This attribute is optional.

##### `type`
Determines whether the active atomic substate(s) of the current state or only its immediate active substate(s) are recorded. This attribute is optional.

#### Valid Children
- [`<transition>`](scxml_user_manual.md#transition-transition)


### Enter Action (`<onentry>`)
A wrapper element containing executable content to be executed when the state is entered

#### Attributes
None

#### Valid Children
- [`<raise>`](scxml_user_manual.md#raise-raise)
- [`<script>`](scxml_user_manual.md#script-script)

#### Example
```
<onentry>
  <raise event="foo"/>
  <raise event="bar"/>
</onentry>
```

### Exit Action (`<onexit>`)
A wrapper element containing executable content to be executed when the state is exited.

#### Attributes
None

#### Valid Children
- [`<raise>`](scxml_user_manual.md#raise-raise)
- [`<script>`](scxml_user_manual.md#script-script)

#### Example
```
<onexit>
  <raise event="foo"/>
  <raise event="bar"/>
</onexit>
```

### Script (`<script>`)

May contain actions according to the [Monkey Brain Core Concepts]((core_concepts.md#actionss).

## Executable Content

"Executable content" refers here basically to sequence [`<raise>`](scxml_user_manual.md#raise-raise) and [`<script>`](scxml_user_manual.md#script-script) elements.

### Raise (`<raise>`)
The `<raise>` element raises an event in the current SCXML session. Note that the event will not be processed until the current block of executable content has completed and all events that are already in the internal event queue have been processed.

#### attributes

##### event
Specifies the name of the event. This attribute is required.

#### Valid Children
None.

#### Example
```
<raise event="foo"/>
```
