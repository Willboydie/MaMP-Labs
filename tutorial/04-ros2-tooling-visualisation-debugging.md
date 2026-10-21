# Module 04: Tooling, visualisation and debugging

> **Part I · ROS 2 foundations** · Est. 8–12 h · Prerequisites: 03 · Track A

## Why this matters / where it lives in this repo

The labs already use the tools: `ros2 run tf2_tools view_frames` (Lab 1, step 1.5),
`ros2 topic echo --once /tf` (step 1.6), RViz configs
([`Lab1/.../config/viz.rviz`](../Lab1/transform_helpers/config/viz.rviz),
[`Lab2/.../cfg/rviz.rviz`](../Lab2/trajectory_demo/cfg/rviz.rviz)), an `rqt_graph`
export ([`Lab1/transform_helpers/rosgraph.png`](../Lab1/transform_helpers/rosgraph.png)),
markers ([`Lab2/assets/lab2/markers.png`](../Lab2/assets/lab2/markers.png)) and
matplotlib plotting after Ctrl-C in Lab 2. Professionals record everything and analyse
offline. This module turns these one-off commands into a workflow.

## Learning objectives

* Use the `ros2` CLI to answer any "what is running and what is it saying?" question in
  under a minute.
* Use `rqt` tools, PlotJuggler and RViz2 effectively, including **fixed frame**, **TF
  prefix**, `RobotModel`, `TF`, `Marker`/`MarkerArray` and `InteractiveMarker`.
* Record and replay data with **rosbag2** (MCAP), with correct QoS for `/tf_static`, and
  analyse bags offline in Python without a running ROS graph.
* Debug crashes and performance problems: `gdb` via launch prefixes, sanitizers, `py-spy`
  and `perf`, plus log-level control.

## Key ideas to own

* **CLI cheat sheet:**
  `ros2 topic list -t` · `ros2 topic info -v` (QoS) · `ros2 topic hz/bw/delay` ·
  `ros2 topic echo --once --field position /joint_states` · `ros2 interface show
  trajectory_msgs/msg/JointTrajectory` · `ros2 node info` · `ros2 param list/get/dump` ·
  `ros2 action list -t` · `ros2 control list_controllers` /
  `list_hardware_interfaces` · `ros2 doctor --report` · `ros2 run tf2_ros tf2_echo base
  fr3_link8` · `ros2 run tf2_tools view_frames`.
* **RViz2 mental model.** Every display draws data *expressed in its header frame*, which
  RViz transforms into the **fixed frame** using TF at the data's **timestamp**. Most "my
  thing doesn't show" problems are one of: wrong fixed frame, a missing TF link, timestamp
  extrapolation, or QoS (e.g. `RobotModel` reads `/robot_description`, which is
  transient-local). Lab 1's **TF prefix** setting points the `RobotModel` at a second,
  prefixed tree (`my_robot/...`) published by your node.
* **Markers.** `visualization_msgs/Marker` and `MarkerArray`: use `LINE_STRIP` for paths,
  `SPHERE_LIST` for waypoints, `TEXT_VIEW_FACING` for labels and `MESH_RESOURCE` for tools.
  Give each marker a unique `(ns, id)`, use `action=DELETEALL` to clear, and use
  `frame_locked` for markers attached to moving frames.
* **rosbag2.** `ros2 bag record -s mcap -o run1 /joint_states /tf /tf_static ...`, then
  `ros2 bag info` and `ros2 bag play --clock`. Prefer MCAP storage: Humble needs the
  `rosbag2_storage_mcap` plugin, and it is the default in later distros. Static transforms
  need transient-local QoS on playback, so learn the `--qos-profile-overrides-path` option.
  Read bags offline with `rosbag2_py`, or without ROS using the `mcap` +
  `mcap-ros2-support` Python packages.
* **PlotJuggler** handles time series much better than `rqt_plot`. It opens bags directly
  and plots fields of any message.
* **Logging.** `RCLCPP_INFO_THROTTLE`, `self.get_logger().debug(...)` (as Lab 1's node
  does), `--ros-args --log-level debug` or per logger `--log-level my_node:=debug`, and
  `rqt_console` to filter.
* **Debugging native code.** In launch, `Node(..., prefix=['gdbserver localhost:3000'])`
  or `prefix=['xterm -e gdb -ex run --args']`; build with
  `-DCMAKE_BUILD_TYPE=RelWithDebInfo`; run sanitizers (`-fsanitize=address,undefined`).
  For Python, `py-spy top --pid` finds where rclpy time goes, and `debugpy` attaches
  VS Code to a running node.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [Recording and playing back data](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) | rosbag2 basics |
| [ros2/rosbag2 README](https://github.com/ros2/rosbag2) | Storage plugins, QoS overrides, Python API |
| [RViz user guide](https://docs.ros.org/en/humble/p/rviz2/doc/index.html) and [Marker display types](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html) | Displays, fixed frame, every marker type |
| [Debugging tf2 problems](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html) | The systematic TF debugging recipe |
| [PlotJuggler](https://github.com/facontidavide/PlotJuggler) | Install it and use it for every time-series question from now on |

**Deeper**

| Resource | Take from it |
|---|---|
| [MCAP](https://mcap.dev/) format and Python libraries | Offline analysis without ROS |
| [Foxglove](https://foxglove.dev/) | Web-based visualisation of live and recorded ROS data |
| [Rerun](https://rerun.io/) | Logging and visualising robotics data from plain Python or C++ (great for algorithm debugging) |
| [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools) | Convenience wrappers for markers, used throughout MoveIt |
| [Logging concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html) | Severity levels, per-logger configuration, output formats |

## Warm-ups

1. **Sixty-second audit.** With Lab 2 running, answer in under a minute each: the rate of
   `/joint_states`; the QoS of `/robot_description`; the joints and interfaces claimed by
   `franka_arm_controller`; the parameters of `robot_state_publisher`.
2. **Break RViz on purpose.** Change the fixed frame to a non-existent frame; publish a
   marker with a stamp 10 s in the future; publish a marker in a frame with no TF path.
   Record each error message and its cause.
3. **Bag round trip.** Record Lab 2 executing the example trajectory (MCAP). Replay it
   with `--clock` while the live system is stopped, and confirm RViz shows the motion
   including static frames. If static frames go missing, fix the QoS override.
4. **Plot in PlotJuggler** all seven joint positions and velocities from the bag, plus the
   error fields of the JTC's state topic (`~/controller_state`, called `~/state` in
   older releases; check `ros2 topic list`). Explain why the error is (nearly) zero with mock
   hardware (Modules 12 and 20 return to this).
5. **Crash on purpose.** Add a null-pointer dereference to a C++ node, launch it under gdb
   via a launch prefix, and get a backtrace. Then find it again with AddressSanitizer.

## Mini project: flight recorder and offline analyser *(A)*

**Part 1: live visualiser node (`mamp_viz`).** Subscribes to TF and the controller state,
and publishes a `MarkerArray` with:
* the end-effector trace as a `LINE_STRIP`, coloured by Cartesian speed;
* commanded waypoints as spheres, with text labels showing their time stamps;
* a translucent **"ghost" robot** at the trajectory's final configuration. Use a second
  `robot_state_publisher` with `frame_prefix: ghost/` and a second `RobotModel` display
  with alpha 0.3.

**Part 2: offline analyser (`mamp_analyze`).** A pure-Python tool (no running ROS graph)
that takes an MCAP bag and produces an HTML report:
* end-effector path reconstructed **from the recorded TF** (compose the transforms yourself);
* joint positions, velocities, accelerations and **jerk** (finite differences with a
  discussion of noise amplification);
* tracking error from the controller state topic;
* violations of the limits in
  [`franka_description/robots/fr3/joint_limits.yaml`](../franka_description/robots/fr3/joint_limits.yaml)
  (position, velocity, and acceleration and jerk if you add them);
* total motion time and path length.

**Proof of competence**

* The offline end-effector path matches a live `tf2_ros` lookup recorded during the same
  run to **< 1e-9 m** at every sample.
* `mamp_analyze run.mcap -o report.html` runs in CI on a small bag committed to your repo,
  and the test fails if you corrupt a joint name in the bag.
* Markers stay attached correctly when you switch RViz's fixed frame between `world` and
  `fr3_link0`.

**Stretch**

* A Foxglove layout or a Rerun viewer for the same data.
* Automatic detection of "IK branch flips": large joint jumps between consecutive
  waypoints while the Cartesian step is small (you'll need this in Module 13).

## Self-check

1. Which three commands tell you a topic's type, its publishers and their QoS, and its rate?
2. RViz shows "No transform from [fr3_link8] to [world]". Give your debugging steps in order.
3. Why can static transforms vanish when replaying a bag, and how do you fix it?
4. What does the TF prefix field in Lab 1's RViz config actually change?
5. How do you get a backtrace from a C++ node that only crashes when started from a launch
   file?
6. Why does differentiating recorded positions twice give a noisy acceleration, and what
   would you do about it?

Next: [Module 05: Rigid-body transformations](05-rigid-body-transformations.md)
