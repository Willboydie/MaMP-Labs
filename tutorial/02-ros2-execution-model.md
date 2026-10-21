# Module 02: Execution model (executors, callbacks, time, lifecycle, real-time)

> **Part I · ROS 2 foundations** · Est. 12–18 h · Prerequisites: 01 · Track B (skim the Key ideas for Track A)

## Why this matters / where it lives in this repo

Look at the bottom of [`traj_demo.py`](../Lab2/trajectory_demo/trajectory_demo/traj_demo.py):

```python
try:
    rclpy.spin(traj_demo)
except KeyboardInterrupt:
    pass
traj_demo.plot_joint_traj()
```

Nothing in a ROS 2 node runs unless an **executor** is spinning it. Your
`joint_state_callback` runs on the executor's thread, and so do the TF listener's
subscriptions in Python (unless you ask for a dedicated thread). Any blocking work in
one callback delays every other callback in the node. Most "my node freezes" and
"lookup_transform always times out" bugs are executor bugs. `ros2_control`'s
controller manager runs a real-time update loop, and you can't write a controller
(Module 12) without understanding what "real-time safe" means.

## Learning objectives

* Explain what `spin` does, and how callbacks are scheduled by the Single- and
  Multi-threaded executors.
* Use **callback groups** (mutually exclusive, reentrant) to make concurrency explicit.
  Diagnose and fix executor deadlocks.
* Handle **time** correctly: ROS time vs system vs steady time, `use_sim_time`, `/clock`,
  wall timers vs ROS timers.
* Write **lifecycle (managed) nodes** and **composable components**, and know when
  intra-process zero-copy applies.
* State the rules of **real-time** code and apply them in C++.

## Key ideas to own

* **Executors.** `rclcpp::spin(node)` creates a `SingleThreadedExecutor`. It waits on the
  middleware, then runs ready callbacks (subscriptions, timers, services, clients, action
  waitables) **one at a time**. A `MultiThreadedExecutor` runs callbacks on a thread
  pool, but only callbacks that their **callback groups** allow to run concurrently.
* **Callback groups.** Every entity belongs to a group, the node's default
  *mutually exclusive* group unless you say otherwise. A mutually exclusive group runs at
  most one of its callbacks at a time. A *reentrant* group may run any number at once,
  including the same callback in parallel. Callbacks in different groups may always run in
  parallel on a multi-threaded executor. Keep the group object alive (store it as a
  member).
* **The classic deadlock.** A callback makes a *synchronous* service call (rclpy
  `client.call(req)`) or waits on a future. The response can only be processed by the
  executor, which is busy running the blocked callback, so it waits forever. This happens
  even if the server lives in another process. Fixes: `call_async` with a done-callback;
  or put the client in a separate callback group and use a multi-threaded executor; or
  restructure the logic as a state machine.
* **The tf2 variant (Python).** `tf2_ros.TransformListener(buffer, node)` in rclpy
  subscribes to `/tf` *on your node's executor* by default. Calling
  `buffer.lookup_transform(..., timeout=Duration(seconds=1))` inside a callback on a
  single-threaded executor blocks the very subscription that would deliver the transform.
  Inside callbacks, ask for the latest available transform without blocking, and handle
  `TransformException`. (The C++ listener spins its own thread by default.)
* **Python is not fast.** rclpy callbacks pay for the GIL, (de)serialisation into Python
  objects and executor overhead. 1 kHz loops in Python are unreliable. That is fine for
  Lab-style orchestration and wrong for control loops.
* **Time.**
  * *ROS time* (`node.get_clock().now()`) follows `/clock` when `use_sim_time:=true`
    (simulators and `ros2 bag play --clock`); otherwise it equals system time.
  * *System time* can jump (NTP). *Steady time* is monotonic; measure durations with it.
  * `create_wall_timer` always uses steady time. `create_timer` (and rclpy's
    `create_timer`) uses the node's clock, so it pauses when simulated time pauses.
  * Stamp a message with the time **the data was valid**, not the time you happened to
    publish it. tf2 interpolates on those stamps (Module 06).
* **Lifecycle nodes.** States: Unconfigured → Inactive → Active → Finalized, with
  transitions `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`. They give
  deterministic bring-up (allocate on configure, start I/O on activate). `ros2_control`
  controllers follow the same state machine: the `spawner` in Lab 2's launch loads,
  configures and activates them.
* **Composition.** Nodes written as components (`rclcpp_components`) can be loaded into
  one process (`component_container`, or `_mt` for multi-threaded). With intra-process
  communication enabled and `std::unique_ptr` publishing, messages pass by pointer:
  zero-copy, and no serialisation.
* **Real-time rules (C++).** In the hot loop: no heap allocation, no unbounded locks,
  no blocking I/O or logging to disk, no exceptions, and nothing with unbounded
  execution time. Preallocate. Use lock-free hand-off (`realtime_tools::RealtimeBuffer`,
  `RealtimePublisher`). Run under `SCHED_FIFO` with `mlockall` on a PREEMPT_RT kernel
  (mainline since Linux 6.12; Ubuntu also ships a real-time kernel). The Franka FCI
  needs a 1 kHz loop, which is why `franka_ros2` requires an RT kernel on real hardware.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [Executors (concepts)](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) | Executor types, callback groups, scheduling semantics |
| [Using callback groups (how-to)](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) | The deadlock examples, in C++ and Python |
| [rclpy: Execution and callbacks](https://docs.ros.org/en/humble/p/rclpy/api/execution_and_callbacks.html) | How rclpy differs from rclcpp |
| [Clock and Time design article](https://design.ros2.org/articles/clock_and_time.html) | ROS vs system vs steady time; sim time |
| [Managed nodes design article](https://design.ros2.org/articles/node_lifecycle.html) | The lifecycle state machine and why it exists |
| [Composition tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) | Components and containers |

**Deeper**

| Resource | Take from it |
|---|---|
| Casini, Blaß, Lütkebohle, Brandenburg, "Response-Time Analysis of ROS 2 Processing Chains under Reservation-Based Scheduling", ECRTS 2019 | The formal model of the executor. Explains surprising starvation behaviour |
| [ROS 2 Real-Time Working Group](https://ros-realtime.github.io/) | RT kernel setup, guidelines, benchmark tools |
| [ros2/ros2_tracing](https://github.com/ros2/ros2_tracing) | LTTng-based tracing of callbacks and messages, for the stretch goal |
| [realtime_tools](https://github.com/ros-controls/realtime_tools) source | How `RealtimeBuffer` and `RealtimePublisher` avoid locks in the RT thread |
| Martín Rico, *A Concise Introduction to Robot Programming with ROS 2*, chapters on executors and lifecycle | C++ worked examples |

## Warm-ups

1. **Make it deadlock, then fix it three ways.** In Python, write a node with a timer
   (1 Hz) that calls a service on the same node with `client.call()`. Watch it hang.
   Fix it with (a) `call_async` and a done-callback; (b) a separate callback group plus a
   `MultiThreadedExecutor`; (c) a different design. Repeat in C++.
2. **Starvation.** Put a 100 Hz timer and a subscription whose callback `sleep`s 200 ms
   into one node. Log the timer's actual periods (steady clock) and plot the histogram.
   Then fix it with callback groups.
3. **Sim time.** Record a bag of `/joint_states` and `/tf` from Lab 2 (Module 04). Play it
   with `--clock` and pause/resume it. Run a node with `use_sim_time:=true` that has
   one ROS timer and one wall timer. Explain what you see.
4. **Lifecycle.** Write a C++ `rclcpp_lifecycle::LifecycleNode` that allocates its
   publisher in `on_configure`, publishes only while active, and releases everything in
   `on_cleanup`. Drive it with `ros2 lifecycle set`.
5. **Zero-copy proof.** Make two components, load them into one container with
   intra-process comms on, publish a `std::unique_ptr` message, and print the message's
   address in publisher and subscriber. Then break zero-copy (e.g. add a second
   subscriber, or publish a `const&`) and explain why the addresses differ.

## Mini project: a deterministic 1 kHz pipeline

Build a C++ composable pipeline that imitates a robot control stack:

`encoder_sim` (1 kHz, noisy joint positions) → `state_estimator` (velocity estimate,
e.g. a 2nd-order filter) → `pd_controller` (torque command) → `actuator_sink`
(records receive timestamps).

Run it in five configurations:

1. Four separate processes.
2. One container, single-threaded executor.
3. One container, multi-threaded executor, explicit callback groups.
4. One container, intra-process zero-copy.
5. Configuration 4 plus RT hygiene: preallocated messages, `SCHED_FIFO` priority, `mlockall`,
   CPU pinning (`taskset`/`isolcpus`), on a PREEMPT_RT kernel if you can.

Run each for 10 minutes, idle and under `stress-ng --cpu $(nproc) --io 2 --vm 2`.

**Proof of competence**

* Histograms of loop period and end-to-end latency (sensor stamp → sink receive) for
  every configuration × load condition. Report p50, p99, p99.9 and max, and count
  **deadline misses** (period > 1.2 ms).
* In your best configuration, **zero deadline misses over 10 minutes under stress** on an RT
  kernel. On a non-RT kernel, report the misses and explain where they come from.
* The same pipeline in rclpy, with a paragraph explaining the gap.

**Stretch**

* Use `ros2_tracing` to attribute latency to middleware, executor wake-up and callback
  execution. Produce a flame-graph-style breakdown.
* Replace the executor with the newer events-based executor (where your distro provides
  one) and compare.

## Self-check

1. Which thread runs your subscription callback in `rclpy.spin(node)`? In
   `rclcpp::spin(node)`?
2. Why does a synchronous service call inside a callback deadlock on a single-threaded
   executor, even when the server is in a different process?
3. Two callbacks in the same mutually exclusive group, on a multi-threaded executor: can they
   run concurrently? What if they are in different groups?
4. Why might `lookup_transform` with a 1 s timeout *always* time out inside a Lab-2-style
   rclpy callback? Give two fixes.
5. When does `create_timer` stop firing but `create_wall_timer` keep going?
6. Name the lifecycle states and the transitions between them. What does the `ros2_control`
   spawner do in those terms?
7. List five things you must never do inside a 1 kHz real-time control loop, and why.

Next: [Module 03: Packages, build, launch and testing](03-ros2-packages-build-launch-test.md)
