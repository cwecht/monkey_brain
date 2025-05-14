# MB is single threaded and deliberately NOT thread-safe

## Context and Problem Statement

Should MB designed to be thread safe?

## Considered Options

* make MB deliberately NOT thread-safe
* make MB thread-safe 

## Decision Outcome

Choose option: "make MB deliberately NOT thread-safe"

### Reasoning

Any kind of threadedness would  unreasonably complicate the implementation of decision engines.
The SCXML state machine implementation is inherently single-threaded.
As each value in the environment may be accessed at any time, each callback would start with a lock and end with a lock.
This would render any kind of threadedness pretty much useless.

### Consequences

 * Good: implementation is simpler
 * Good: no performance overhead due to locks
 * Bad: MB can not be used with
   [MultithreadedExecutor](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1executors_1_1MultiThreadedExecutor.html)
