# Monkey Brain

<p align="center">
  <img src="doc/logo.svg" alt="monkey brain logo" width="250">
</p>

**Monkey Brain** started out as a _finite state machine_ based on [SCXML](https://www.w3.org/TR/scxml/)
but is now a *decision engine framework* which could potential employ engines like rule engines or behaviour
trees.

> [!WARNING]
> This is an early version of monkey brain. There might be missing features which seem obvious.
> Nevertheless, in case you encounter any issue or miss an important feature, please, feel free to
> create an [issue](https://github.com/cwecht/monkey_brain/issues).

## Key Features
 - **ROS2 Integration**: Integrates with ROS2 but is _not required_ (migration to other middleware systems should be
   pretty easy).
 - **No code required**: state machine is based on the [State Chart extensible Markup Language
   (SCXML)](https://en.wikipedia.org/wiki/SCXML) developed by [W3C](https://www.w3.org/TR/scxml/). No Python or C++ is
   required!
 - **Rapid Prototyping**: no recompilation required on changes. State machine is loaded during startup. Changes on the
   state machine only require a restart.
 - **Extensibility**: Ships with support for ROS2 publishers, subscribers, services and action clients. But custom
   actions and functions can be added easily using the comprehensive plugin system.
 - **Web Viewer**: support for [YASMIN's Viewer](https://uleroboticsgroup.github.io/yasmin/4.0.1/yasmin_viewer.html) is
   available.
 - **Just the start**: do you need something else than a state machine (e.g. rules engines or behavior trees)?
   Monkey Brain can act as the starting point for that as well. Create your own _decision engine_ fitting
   into the Monkey Brain Framework.

## Demos

Monkey Brain ships with a few demo applications. This is a quick start guide for running those demos.
Please note that those demos are based on [YASMIN's demos](https://github.com/uleroboticsgroup/yasmin/tree/e5f6a9e889ac1f00af57a5b7c592fc22814d9a45?tab=readme-ov-file#demos).

### Build

Monkey Brain is pretty light on dependencies. For a basic setup "only" `ros-jazzy-desktop` is required.
For the demos though, `ros-jazzy-yasmin-demos` is required.

For the demos a few other packages need to be build as part of your workspace. This can easily be archived like this:

```
wget https://raw.githubusercontent.com/cwecht/monkey_brain/refs/heads/main/demos.repos
vcs import src < demo.repos
colcon build
```
### Run

#### Atomic State Machine

This demo is inspired by [SMACC2's `sm_atomic` demo](https://github.com/robosoft-ai/SMACC2/tree/jazzy/smacc2_sm_reference_library/sm_atomic)
and [YASMIN'S Vanilla Demo](https://github.com/robosoft-ai/SMACC2/tree/jazzy/smacc2_sm_reference_library/sm_atomic).
But is also showcases how to use a simple publisher.

```
ros2 launch monkey_brain_demos atomic_launch.py
```

<details>
<summary>Click to expand</summary>

```XML
<?xml version="1.0" encoding="UTF-8"?>
<scxml xmlns="http://www.w3.org/2005/07/scxml" version="1.0" initial="one">
  <state id="one">
    <onentry><script>PERFORM /timeout_timer/reset_timer</script></onentry>
	<transition event="/timeout_timer" target="two">
       <script>/out_topic/data := true; PERFORM /out_topic/publish; PERFORM /logger/print</script>
	</transition>
  </state>
  <state id="two">
    <onentry><script>PERFORM /timeout_timer/reset_timer</script></onentry>
    <transition event="/timeout_timer" target="one">
      <script>/out_topic/data := false; PERFORM /out_topic/publish</script>
    </transition>
  </state>
</scxml>
```

```YAML
io: [
        { name: "example_interfaces/Bool/Publisher",
          topic: "/out_topic"
        },
        { name: "oneshot_timer/Inputter",
          topic: "/timeout_timer",
          params: { timeout: 1000 }
        },
        { name: "basic_logger/Logger",
          topic: "/logger",
          params: { format: "Hello World!!", severity: 20, params: {}}
        },
]

operators: []
```

</details>

#### Service Client Demo

This demo is inspied by
[YASMIN's Service Demo](https://github.com/uleroboticsgroup/yasmin/tree/e5f6a9e889ac1f00af57a5b7c592fc22814d9a45?tab=readme-ov-file#service-demo-fsm--ros-2-service-client).

```
ros2 launch monkey_brain_demos service_client_demo.launch.py
```

<details>
<summary>Click to expand</summary>

```XML
<?xml version="1.0" encoding="UTF-8"?>
<scxml xmlns="http://www.w3.org/2005/07/scxml" version="1.0" init="add_two_ints_server_unavailable">
    <state id="add_two_ints_server_unavailable">
        <transition event="/add_two_ints/is_ready" cond="/add_two_ints/is_ready" target="add_two_ints_server_is_ready"/>
    </state> 
    <state id="add_two_ints_server_is_ready">
        <onentry><script>
           /add_two_ints/request/a := 1
           /add_two_ints/request/b := -5
           PERFORM /add_two_ints/call
        </script></onentry>
        <transition event="/add_two_ints/response" target="answer_received">
            <script>
                /log_sum/sum := /add_two_ints/response/sum
                PERFORM /log_sum/print
            </script>
        </transition>
    </state> 
    <final id="answer_received"/>
</scxml>
```

```YAML
io: [
        { name: "example_interfaces/AddTwoInts/ServiceClient",
          topic: "/add_two_ints",
          params: { retry_period: 2000 }
        },
        { name: "basic_logger/Logger",
          topic: "/log_sum",
          params: { format: "Sum: {}", severity: 20, params: { "sum": "int64" }}
        }
]
operators: []
```

</details>

#### Action Client Demo

This demo is inspied by
[YASMIN's Action Demo](https://github.com/uleroboticsgroup/yasmin/tree/e5f6a9e889ac1f00af57a5b7c592fc22814d9a45?tab=readme-ov-file#action-demo-fsm--ros-2-action).

```
ros2 launch monkey_brain_demos action_client_demo.launch.py
```

<details>
<summary>Click to expand</summary>

```XML
<?xml version="1.0" encoding="UTF-8"?>
<scxml xmlns="http://www.w3.org/2005/07/scxml" version="1.0" init="fibonacci_server_unavailable">
    <state id="fibonacci_server_unavailable">
        <transition event="/fibonacci/is_ready" cond="/fibonacci/is_ready" target="fibonacci_server_is_ready"/>
    </state> 
    <state id="fibonacci_server_is_ready">
        <onentry><script>
           /fibonacci/goal/order := 20
           PERFORM /fibonacci/send_goal
        </script></onentry>
        <transition event="/fibonacci/result" target="answer_received">
            <script>
                /log_sequence/sequence := /fibonacci/result/sequence
                PERFORM /log_sequence/print
            </script>
        </transition>
    </state> 
    <final id="answer_received"/>
</scxml>
```

```YAML
io: [
        { name: "example_interfaces/Fibonacci/ActionClient",
          topic: "/fibonacci",
          params: { retry_period: 500 }
        },
        { name: "basic_logger/Logger",
          topic: "/log_sequence",
          params: { format: "Result: {}", severity: 20, params: { "sequence": "int32[]" }}
        }
]
operators: []
```

</details>

#### Visualisation

A running state machine as well as the currently active states can be visualized using
[YASMIN Viewer](https://github.com/uleroboticsgroup/yasmin/tree/e5f6a9e889ac1f00af57a5b7c592fc22814d9a45?tab=readme-ov-file#yasmin-viewer).

In order to do that, `monkey_brain_node` must be configured such that `yasmin_state_obser` is loaded.
The example shows an example launch file. 

```Python
Node(
    package='monkey_brain_node',
    executable='monkey_brain_node',
    name='mb_atomic',
    namespace='mb_atomic',
    parameters=[
      {'path_to_scxml': os.path.join(config_path, 'state_machine.scxml')},
      {'path_to_ios': os.path.join(config_path, 'plugins.yaml')},
      {'state_machine_observer': 'yasmin_state_observer' },
    ],
    ros_arguments=['--log-level', 'info'],
    output='screen'
)
```

`yasmin_viewer_node` will now be able to visualise monkey brains state machine.

```
ros2 run yasmin_viewer yasmin_viewer_node
```


## Information

 - [Tutorial](doc/tutorial.md) shows how to build [Service Client Demo](README.md#service-client-demo) from scratch.
 - [Core Concepts](doc/core_concepts.md) explains the core concepts behind monkey brain.
 - [SCXML User Manual](scxml_user_manual.md) explains SCXML and the feature subset which is supported by monkey brain.

## Special Thanks

 - [SMACC2](https://github.com/robosoft-ai/SMACC2) for showing relevant usecases.
 - [YASMIN](https://github.com/uleroboticsgroup/yasmin) for providing reasonable examples and the [YASMIN Viewer](https://uleroboticsgroup.github.io/yasmin/4.0.1/yasmin_viewer.html).
 - [scxmlxx](https://github.com/jp-embedded/scxmlcc) for introducing me to SCML.
 - W3 for providing a [reference implementation of a SCXML state machine](https://www.w3.org/TR/scxml/#AlgorithmforSCXMLInterpretation) and [conformance tests](https://www.w3.org/Voice/2013/scxml-irp/#h_results).
