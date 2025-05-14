# Monkey Brain Core Concepts

Monkey Brain has a few core concepts which must be understood, before it can be used properly.

## Environment

The environment is monkey brain's view of the outside world.
It consists of
  * values it can read from,
  * values it can write to,
  * events and
  * perform actions.

## IO Plugins

The environment is defined by the `plugins.yaml`.
In that file io plugins are loaded and configured which then can introduce values, events or perform actions.

## Names

Values, events and perform actions are referred to by their names.
Those names are similar to ROS2 names and usually composed from
  * the full name of the monkey brain node,
  * the name/topic of the plugin instance and
  * a identifier.

Let's consider the following example. 
We'll assume, that the fully qualified name of the node is `/behaviour/state_machine'.
Further we'll assume, that the following `plugins.yaml'.

## Example
```YAML
io: [
        { name: "example_interfaces/Bool/Subscriber",
          topic: "input_bool",
        },
        { name: "example_interfaces/Bool/Publisher",
          topic: "output_bool",
        }

]
operators: []
```

Configured with this `plugins.yaml` `monkey_brain_node` will create
 * a subscriber on the topic `/behaviour/state_machine/input_bool` and
 * a publisher on the topic `/behaviour/state_machine/output_bool`.

Monkey brain's environment will look like this:
 * the following values are available:
   * `/behaviour/state_machine/input_bool` - readonly of type `example_interfaces/Bool`
   * `/behaviour/state_machine/input_bool/data` - readonly of type `bool`
   * `/behaviour/state_machine/output_bool` - writeonly of type `example_interfaces/Bool`
   * `/behaviour/state_machine/output_bool/data` - writeonly of type `bool`
 * the following events are available:
   * `/behaviour/state_machine/input_bool` - will be triggered when a new message arrives.
 * the following perform actions are available:
   * `/behaviour/state_machine/output_bool/publish` - can be used to publish a message via `/behaviour/state_machine/output_bool`.

## Events

Events allow monkey brain to react to events reaching `monkey_brain_node` from the outside world.
Typical examples are:
  * the arrival of a message on a subscriber,
  * the response to a service call or
  * a timer event.

## Conditions

In monkey brain the user can define conditions on values of the environment.
Those conditions are [first-order logical expressions](https://en.wikipedia.org/wiki/First-order_logic).
The syntax is more or less comparable to C, C++ or Python.
The most significant difference is, that slashes ("/") are allowed in variable names and
that tokens (except brackets "()") must be separated by at least one whitespace.

The following operators are supported by default: `NOT`, `!`, `AND`, `&&`, `OR`, `||`, `==`, `!=`, `<`, `>`, `>=`, `<=`, `<=`.
Operands of those operators can be
  * values,
  * literals (supported are integers, floating point numbers, strings, chars and arrays of those) and
  * the result of an expression.

Example coniditions could look like this:
  * `/behaviour/state_machine/input_bool/data`
  * `NOT /behaviour/state_machine/input_bool/data`
  * `NOT (/behaviour/state_machine/input_bool/data AND /behaviour/state_machine/input_bool2/data)`
  * `/behaviour/state_machine/input_int/a/data < 5`

Values, literals and results have a type.
The operands of an operation must have matching types.
If this isn't the case, the value must be casted.
The following casts are possible:
  * `SIGNED` - converts to a signed integer,
  * `UNSIGNED` - converts to a unsigned integer and
  * `FLOAT` - converts to a flating point number.

The syntax for casts is function-like. Examples could be:
  * `SIGNED(/a_unsigned_int)`,
  * `UNSIGNED(/a_float)`,
  * `FLOAT(/a_signed_int)`,

Please note, that monkey brain performs all computations in
  * 64-bit signed integers,
  * 64-bit unsigned integers and
  * 64-bit floating-point numbers.
This is the reason why only those three casting operations are sufficient.

Please note, that those casting operations only work between numbers ((un)signed integers and floating-point).
Casting from and to e.g., `bool`, `char` or `string` is not supported.

The types of literals are derived like this
 * integer numbers like `-1` or `5` are signed integers.
 * integers with a `U` suffix like `1U` or `5U` are unsigned integers.
 * numbers with a dot (`.`) like `5.0` or `4.2` are floating-point numbers.
 * everything within `""` are strings.
 * a character in `''` like `'a'` is a `char`.
 * squarbrackets and comma separated literals denote an array, e.g. `[1, 2, 5]` is an arraqy of signed integers.

### Operator Plugins

monkey brain can be extended with custom operators. 
`monkey_tool list_operator_plugins` will tell us which operator plugins are available in the current workspace.
By default, its output will tell us that two operators are available:
```
2 available operators:
 - math/abs
 - math/sqrt
```

`monkey_tool show_operator <plugin name>` will tell us some details about the given plugin.
For example, `monkey_tool show_operator_plugin math/abs` will result in:
```
Unary function that takes a number and returns its absolute value. Works for fixed-point and floating point numbers.
```

In order to be used, operator plugins must be added to the `operators` list in `plugins.yaml' e.g., like this:
```YAML
io: []
operators: ["math/abs"]
```

With that, "math/abs" can be used just as any other function (see the cast-operator examples above).

## Actions

Monkey brain can not only perceive the environment but only influence it.
This is what actions are for.
Currently, two actions are supported.

### Assign Action

The assign action allows to assign values to writable values.
The syntax looks like this:
```
/target := /any_value
```

The token `:=` identifies this line as an assign action.
On the left side of this statement any writable value may reside.
On the right side, any expression which yields a matching value may reside.
The syntax of those expression is identical to the syntax of condition above
and the same rules apply.

These are further examples of assign action statements:
 * `/target_int := 5`
 * `/target_int := 5 + /source_int`
 * `/target_float := FLOAT(/source_int * /source_int)`
 * `/target_float := math/abs(/source_int * /source_int)`

### PERFORM Action

With perform actions, behaviour outside of monkey brain can be triggered.
The syntax looks like this:
```
PERFORM /behaviour/state_machine/output_bool/publish
```

This example refers to the example above.
In that context, this statement will cause the publication of a message
on the `/behaviour/state_machine/output_bool` topic.

### Action Sequences

It is possible to put multiple action statements in sequence.
For that the individual action statements must be separated by a
semicolon or a newline.
Hence, the following action sequences are both possible.
```
/behaviour/state_machine/output_bool/data := true
PERFORM /behaviour/state_machine/output_bool/publish
```
```
/behaviour/state_machine/output_bool/data := true; PERFORM /behaviour/state_machine/output_bool/publish
```

