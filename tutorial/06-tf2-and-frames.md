# Module 06: tf2 and coordinate frames in ROS

> **Part II · Modelling maths** · Est. 12–16 h · Prerequisites: 01, 05 · Track A

## Why this matters / where it lives in this repo

[Lab 1's README](../Lab1/README.md) is essentially an introduction to tf2. Its node
subscribes to joint states and **broadcasts** a parallel tree under `my_robot/`. Lab 2's
`get_ee_pos_ros` **listens** to TF to get the end-effector position, and its launch file
publishes a **static** `world → base` transform. `robot_state_publisher`, the
centrepiece of both labs, is a tf2 broadcaster driven by the URDF and `/joint_states`.

## Learning objectives

* Explain tf2's data model: a **time-buffered tree** of transforms assembled from
  many publishers, queried by any node.
* Use `TransformBroadcaster`, `StaticTransformBroadcaster`, `TransformListener` +
  `Buffer`, `buffer.transform(...)` and `tf2_ros::MessageFilter` in C++ and Python.
* Get `lookup_transform` argument order and semantics right every time, including
  **time travel**.
* Handle `LookupException`, `ConnectivityException` and `ExtrapolationException`
  deliberately.
* Apply ROS frame conventions (REP 103 and REP 105) and stamp derived data correctly.

## Key ideas to own

* **Tree, not graph.** Each frame has exactly one parent at a given time, so the path
  between any two frames is unique (the README's "tf tree" section explains why). Each
  message on `/tf` is a `TransformStamped` meaning "pose of `child_frame_id` in
  `header.frame_id` at `header.stamp`".
* **`/tf` vs `/tf_static`.** Dynamic transforms are volatile and must be republished
  continuously; tf2 keeps a short history (10 s by default) per edge. Static transforms
  are published once with **transient-local** QoS and are valid for all time.
  `robot_state_publisher` publishes fixed joints on `/tf_static` and movable joints on
  `/tf`.
* **`lookup_transform(target, source, time)`** returns ${}^{target}T_{source}$: the pose of
  `source` in `target`, which also maps coordinates *from* `source` *into* `target`. Say
  "target ← source" out loud every time you type it.
* **Time.** tf2 walks the path between the two frames and evaluates every edge at the
  requested time. It interpolates between stamps (linear for translation, slerp for
  rotation). It throws `ExtrapolationException` outside an edge's buffered interval, so
  it never extrapolates. `Time()` (zero) means *latest common time*: the newest time
  at which **every** edge on the path has data. That is not "now".
* **Time travel:** `lookup_transform(target, t_target, source, t_source, fixed_frame)`
  answers "where was `source` at `t_source`, expressed in `target` at `t_target`?". The
  `fixed_frame` must not move between the two times.
* **Stamping derived data.** A transform computed from a `JointState` should carry *that
  message's* stamp, not `now()` at publish time. Otherwise consumers combine data from
  different instants, and fast motions show lag or extrapolation errors.
* **Python listener caveat** (Module 02): the rclpy `TransformListener` spins on your
  node's executor by default. Don't block waiting for a transform inside a callback on a
  single-threaded executor.
* **Conventions.** [REP 103](https://www.ros.org/reps/rep-0103.html): SI units,
  right-handed frames, x forward / y left / z up for bodies, and z forward / x right / y
  down for camera *optical* frames. [REP 105](https://www.ros.org/reps/rep-0105.html):
  `earth → map → odom → base_link` for mobile robots. A fixed manipulator usually hangs
  off `world`, as Lab 2's static transform does.
* **Namespacing frames.** ROS 1's `tf_prefix` no longer exists. In ROS 2,
  `robot_state_publisher` has a `frame_prefix` parameter, and Lab 1 builds the prefix into
  the frame names itself.
* **When not to use tf2.** For a 1 kHz controller or a planner doing 10⁶ FK evaluations,
  compute kinematics directly (your own code, KDL or Pinocchio). tf2 is for sharing frames
  between processes, not for inner loops.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [Introduction to tf2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) and the rest of the [tf2 tutorial track](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html): static and dynamic broadcasters, listener, adding a frame, time, time travel, debugging, quaternion fundamentals | Do every tutorial **in both C++ and Python** |
| [tf2 concepts page](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html) | Design and the buffer model |
| [REP 103](https://www.ros.org/reps/rep-0103.html) and [REP 105](https://www.ros.org/reps/rep-0105.html) | Conventions everyone assumes you follow |

**Deeper**

| Resource | Take from it |
|---|---|
| T. Foote, "tf: The transform library", IEEE TePRA 2013 | Original design rationale: spanning set, caching, time |
| [ros2/geometry2](https://github.com/ros2/geometry2) source: `tf2/src/buffer_core.cpp`, `tf2/src/cache.cpp` | The real algorithm. Read it before the mini project, then again after |
| [robot_state_publisher](https://github.com/ros/robot_state_publisher) source | How the URDF becomes `/tf` + `/tf_static` (it uses KDL internally) |

## Warm-ups

1. **Inspect.** Run `ros2 run tf2_ros tf2_echo base fr3_link8` and
   `ros2 run tf2_ros tf2_monitor` with Lab 1 running. What are the publish rates, and who
   is the authority for each frame?
2. **Orbiting carrot.** Write a C++ broadcaster for a frame `carrot` orbiting `fr3_link8`,
   and a Python listener that prints the carrot in `base` coordinates at 10 Hz,
   non-blocking, with proper exception handling.
3. **Direction drill.** Look up both `base → fr3_link8` and `fr3_link8 → base`. Verify one
   is the inverse of the other, then transform a `PointStamped` from the gripper frame
   into `base` with `buffer.transform`.
4. **Time travel.** Report where `fr3_link8` was 2 seconds ago, expressed in the
   *current* `fr3_link8` frame, with `world` as the fixed frame.
5. **Static vs dynamic.** With Lab 1 running, kill `robot_state_publisher`. Which frames
   disappear from RViz, which persist, and after how long? Explain using QoS and buffer
   length.
6. **Stamp experiment.** Publish a derived frame stamped (a) with `now()` and (b) with the
   source `JointState` stamp, while the arm moves fast. Show the difference with
   `tf2_echo` or PlotJuggler.

## Mini project: tf2 from scratch *(A: Python version; B: C++ version)*

Implement your own transform buffer (`mamp_tf_scratch`) and prove it matches tf2.

**Requirements**

1. Subscribe to `/tf` and `/tf_static` with the correct QoS for each.
2. Keep a time-sorted, length-bounded cache per child frame, and an unbounded static store.
   Support a frame changing parent over time.
3. `lookup(target, source, time)`: find the path through the lowest common ancestor,
   evaluate each edge at `time` with lerp and slerp, and compose. Raise errors with
   tf2's semantics: connectivity, lookup and extrapolation, with tf2's rules for static
   edges and `Time()`/latest common time.
4. `lookup_full(target, t_target, source, t_source, fixed_frame)` (time travel).
5. `to_dot()`: export the current tree with per-edge rate, latest stamp and buffer
   length, like `view_frames`.
6. Use your `mamp_liegroups` (Module 05) for all the maths.

**Proof of competence**

* On a recorded Lab 2 bag replayed with `--clock`, 10,000 random
  `(target, source, time)` queries agree with `tf2_ros.Buffer` to **≤ 1e-9** in
  translation and rotation angle, and **every** query that tf2 rejects is rejected by
  yours with the same exception type.
* A unit-test suite that builds synthetic trees (re-parenting, static + dynamic mixes,
  gaps in data) without any ROS process running.
* A benchmark of lookup latency against `tf2_ros` (Python and C++) for path lengths 1–10.

**Stretch**

* **TF lint:** a node that flags non-unit quaternions, frames stamped in the future,
  stale dynamic frames, two publishers fighting over one child frame, and
  `/tf_static` publishers with the wrong durability.
* Port the buffer to C++ with lock-free reads for concurrent queries.

## Self-check

1. What exactly does `lookup_transform("base", "fr3_link8", t)` return, and which way does
   it map a point?
2. Why must TF be a tree? What happens when two nodes publish the same child frame with
   different parents?
3. Contrast `/tf` and `/tf_static`: QoS, semantics, and how tf2 stores each.
4. How does tf2 evaluate a transform at a time between two samples, and after the newest one?
5. What does `Time()` mean in a lookup, and why is it not "now"?
6. Why should a transform derived from a `JointState` carry that message's stamp?
7. Name two situations where you should bypass tf2 and compute kinematics directly.

Next: [Module 07: Robot description](07-robot-description-urdf-xacro.md)
