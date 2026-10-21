# Module 12: Robot control and `ros2_control`

> **Part IV · Dynamics and control** · Est. 25–35 h · Prerequisites: 02, 11 (the `ros2_control` half needs only 01–03) · Track A: the `ros2_control` half; Track B: everything

## Why this matters / where it lives in this repo

Lab 2's bring-up is a complete `ros2_control` stack:

* [`cfg/ros2_controllers.yaml`](../Lab2/trajectory_demo/cfg/ros2_controllers.yaml): a
  controller manager at **100 Hz** running a `JointTrajectoryController`
  (`franka_arm_controller`, position command interface; position + velocity state) and a
  `JointStateBroadcaster`.
* [`urdfs/fr3.urdf`](../franka_description/urdfs/fr3.urdf): a `<ros2_control>` block with
  `mock_components/GenericSystem`, which simply **mirrors commands into states**.
* [`launch/bringup.launch.py`](../Lab2/trajectory_demo/launch/bringup.launch.py):
  `ros2_control_node` plus two `spawner`s.
* [`franka_arm.ros2_control.xacro`](../franka_description/robots/common/franka_arm.ros2_control.xacro):
  the real-robot version, with effort, velocity and position command interfaces and a
  switch between fake, Gazebo and real (`franka_hardware/FrankaHardwareInterface`)
  plugins.

Because the hardware is a mirror, Lab 2's "tracking" is perfect by construction: you are
plotting the JTC's interpolated reference, not a robot's response. This module explains
the machinery (the `ros2_control` half) and then the control theory you need once real
physics is involved.

## Learning objectives

* Explain the `ros2_control` architecture: controller manager, resource manager,
  hardware components, controllers, broadcasters, command and state interfaces, the
  read → update → write loop, and controller lifecycle and switching.
* Know exactly what the JTC does with a trajectory: interpolation by available fields,
  tolerances, goal handling and preemption.
* Write a **C++ controller plugin** and a **hardware component plugin** that are
  real-time safe.
* Design and analyse joint-space controllers (PD, PD + gravity compensation, computed
  torque) and task-space controllers (operational space, impedance, admittance).

## Key ideas to own

### `ros2_control` (Track A)

* **The loop.** At `update_rate`, the controller manager calls `read()` on all hardware,
  then `update(time, period)` on every active controller in order, then `write()`. That
  loop runs in a dedicated (ideally real-time) thread. Everything else (topics,
  parameters, services) talks to it through real-time-safe buffers.
* **Interfaces.** Hardware exports *state interfaces* (e.g. `fr3_joint1/position`) and
  *command interfaces* (e.g. `fr3_joint1/position`). A controller **claims** command
  interfaces exclusively, so two controllers can't command the same joint, and switching
  controllers swaps claims atomically. Broadcasters only read states.
* **Hardware components** are pluginlib plugins (`SystemInterface`,
  `ActuatorInterface`, `SensorInterface`) with a lifecycle (`on_init`, `on_configure`,
  `on_activate`, …) plus `read()` and `write()`. `mock_components/GenericSystem` copies
  commands to states (with options for offsets and mirroring sensor commands). It's
  perfect for testing plumbing and useless for testing control.
* **Controllers** are plugins with `command_interface_configuration()`,
  `state_interface_configuration()`, lifecycle callbacks, and `update()`.
  *Chainable* controllers expose reference interfaces to other controllers (e.g. an
  admittance controller feeding a JTC).
* **JTC semantics.** A trajectory replaces the active one, from its stamp onwards (stamp
  zero means now). Interpolation depends on the fields provided: linear with positions
  only, cubic with positions + velocities, quintic with accelerations too. The JTC
  checks path and goal tolerances (configurable per joint) and reports errors through
  the **action** result, which the topic interface never reports. With a velocity or
  effort command interface it closes a PID loop around the reference.
* **CLI:** `ros2 control list_controllers`, `list_hardware_interfaces`,
  `switch_controllers --activate a --deactivate b`, `load_controller`.

### Control theory (Track B)

* **PD + gravity compensation.** $\tau = K_p e - K_d\dot q + g(q)$ is globally
  asymptotically stable for set-points. The proof uses
  $V = \tfrac12\dot q^\top M\dot q + \tfrac12 e^\top K_p e$, the skew-symmetry of
  $\dot M - 2C$, and LaSalle.
* **Computed torque (feedback linearisation).**
  $\tau = M(q)\big(\ddot q_d + K_d\dot e + K_p e\big) + C\dot q + g$ gives linear,
  decoupled error dynamics **if the model is exact**. Study what model error does to it,
  and how robust or adaptive variants (Slotine–Li) fix it.
* **Operational space control (Khatib).** $\Lambda = (JM^{-1}J^\top)^{-1}$ and
  $\tau = J^\top\big(\Lambda(\ddot x_d + K_d\dot e_x + K_p e_x) + \mu + p\big) + N^\top\tau_0$,
  where $N$ is the *dynamically consistent* null-space projector. That gives redundant
  arms like the FR3 a posture task "for free".
* **Impedance vs admittance (Hogan).** Impedance control makes the robot *behave like* a
  mass–spring–damper towards external forces: measure motion, command force or torque.
  This is the Franka's native mode, and franka_ros2 ships a Cartesian impedance example.
  Admittance control measures force and commands motion, for stiff position-controlled
  industrial arms.
* **Practicalities.** Discretisation and delay, torque saturation, anti-windup,
  velocity estimation (filtered differentiation), and why gains that work in simulation
  oscillate on hardware.

## Study materials

**Core: `ros2_control`**

| Resource | Take from it |
|---|---|
| [ros2_control documentation (Humble)](https://control.ros.org/humble/index.html), especially the [Controller Manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html) and [Mock Components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) pages | Architecture, loop, interfaces, CLI |
| [joint_trajectory_controller user docs](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) | Interpolation, tolerances, interface combinations, parameters |
| [Writing a hardware component](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html) | Plugin skeleton, lifecycle, export |
| [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos), examples 1 (RRBot), 2 (DiffBot) and 7 (full 6-DoF tutorial with a custom controller) | Working code to copy patterns from |

**Core: control theory**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 11 + videos | Motion control, force control, hybrid and impedance control |
| Siciliano et al., Ch. 8 (motion control) and Ch. 9 (force control) | Decentralised and centralised control, impedance, admittance, hybrid |
| Brian Douglas, *Control Systems Lectures* / MATLAB Tech Talks (YouTube) | Intuition for PID, stability and state space, if control is new to you |
| Åström & Murray, *Feedback Systems* ([free at fbswiki.org](https://fbswiki.org/)) | Rigorous but readable classical and state-space control |

**Deeper**

| Resource | Take from it |
|---|---|
| O. Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation", IEEE J. Robotics & Automation, 1987 | OSC from the source |
| N. Hogan, "Impedance Control: An Approach to Manipulation", Parts I–III, ASME J. Dyn. Syst. Meas. Control, 1985 | Impedance from the source |
| J.-J. Slotine & W. Li, *Applied Nonlinear Control* (1991), Ch. 9 | Lyapunov design, adaptive manipulator control |
| C. Ott, *Cartesian Impedance Control of Redundant and Flexible-Joint Robots* (Springer, 2008) | The theory behind DLR/Franka-style impedance control |
| [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2) example controllers | Real impedance and torque controllers for this robot |
| [realtime_tools](https://github.com/ros-controls/realtime_tools) | `RealtimeBuffer`, `RealtimePublisher` |

## Warm-ups

1. **Interfaces.** With Lab 2 running, list the hardware interfaces and which controller
   claims each. Try to activate a second position controller on the same joints and read
   the error.
2. **Interpolation types.** Send the same three-point trajectory to the JTC (via the
   action) with (a) positions only, (b) positions + velocities, (c) positions +
   velocities + accelerations. Record the reference from the JTC's state topic and plot
   position, velocity and acceleration. Identify linear, cubic and quintic behaviour, and
   the velocity discontinuities in (a).
3. **The mirror.** Read `mock_components/GenericSystem`'s source (in
   `ros2_control/hardware_interface`). Explain precisely why Lab 2's plotted trajectory
   says nothing about how a real arm would track it.
4. **Lyapunov on paper.** Prove PD + gravity compensation is globally asymptotically
   stable, and point to the step where $\dot M - 2C$ skew-symmetry is used.
5. **Read a real controller.** Walk through franka_ros2's Cartesian impedance example:
   identify the stiffness and damping, the null-space term and how torques are commanded.

## Mini project: controllers and hardware, from scratch

### Part 1: a real-time-safe trajectory controller plugin *(A)*

Write `mamp_controllers/QuinticTrajectoryController` (C++, pluginlib). It:
* claims position command interfaces, plus position and velocity states, for a
  configurable joint list;
* accepts `JointTrajectory` on a topic, handed to the RT thread via
  `realtime_tools::RealtimeBuffer`;
* interpolates with quintic segments, enforcing velocity and acceleration limits from
  parameters by stretching segment times;
* publishes its reference and error with a `RealtimePublisher`.

Load it in `mamp_bringup` alongside the stock JTC and switch between them at runtime.

**Proof:** (i) with positions + velocities + accelerations given, your reference matches
the JTC's to 1e-9; (ii) `update()` performs **zero heap allocations**, shown by a gtest
harness that calls `update()` 10⁵ times under an allocation-counting `operator new`
hook; (iii) switching controllers mid-motion produces no position jump.

### Part 2: your physics engine as hardware *(B)*

Write `mamp_sim_hardware`, a `SystemInterface` plugin wrapping your Module 11 simulator.
It exposes **effort** command interfaces and position, velocity and effort states, steps
the physics in `read()`/`write()` (with substeps), and offers a service to apply an
external wrench to any link. Then write three torque controllers as plugins:

1. PD + gravity compensation (set-points).
2. Computed-torque tracking.
3. Cartesian impedance with a null-space posture task.

**Experiments**
* Track a 4 s figure-of-eight in Cartesian space with (a) an exact model and (b) ±20%
  link-mass error plus unmodelled Coulomb friction. Report RMS and maximum tracking error
  and peak torques against the effort limits.
* Apply a static 10 N push to the end effector under impedance control and measure the
  displacement in 3 directions.

**Proof of competence**
* No controller diverges in any experiment; plots of error and torque are included.
* Computed torque with an exact model tracks to within numerical-integration error, and
  its degradation under model error is quantified and explained.
* Measured static stiffness under impedance control is within **5%** of the commanded
  $K$ in each direction, with the null-space task demonstrably not affecting it.

**Stretch**
* Operational-space control with a dynamically consistent null space. Compare it with
  impedance on the same tasks.
* Slotine–Li adaptive control that identifies an unknown payload mass online.
* Run the identical controllers in Gazebo and MuJoCo through their `ros2_control` plugins
  (Module 20) and explain the differences.

## Self-check

1. Describe one iteration of the controller manager loop. What must be real-time safe,
   and how do topics reach the RT thread?
2. Why can't two controllers claim the same command interface? How does switching work?
3. What does `mock_components/GenericSystem` do, and what does it imply about Lab 2's
   plots?
4. Which interpolation does the JTC use for positions-only points, and what is the
   consequence at waypoints?
5. Sketch the Lyapunov proof for PD + gravity compensation.
6. What happens to computed torque under model error? Name two remedies.
7. Impedance vs admittance: which suits a Franka, which suits a stiff industrial arm, and
   why?
8. Why must `update()` never allocate or lock?

Next: [Module 13: Trajectory generation](13-trajectory-generation.md)
