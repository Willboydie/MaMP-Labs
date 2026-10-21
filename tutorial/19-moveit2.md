# Module 19: MoveIt 2

> **Part VII · Frameworks and simulation** · Est. 20–30 h · Prerequisites: 10, 12, 16 · Track A

## Why this matters / where it lives in this repo

MoveIt 2 is the ROS 2 manipulation framework. It glues together everything you've
learned: the URDF (Module 07), kinematics plugins (Module 10), collision checking
(Module 15), planners (Modules 16–18), time parameterisation (Module 13) and execution
through `ros2_control`'s `FollowJointTrajectory` (Module 12). Lab 2 builds a tiny,
collision-unaware version of this pipeline by hand. MoveIt is the production version,
and knowing its internals (rather than just clicking "Plan" in RViz) is what lets you
extend it, debug it and benchmark against it.

## Learning objectives

* Explain MoveIt 2's architecture: `move_group` / MoveItCpp, the robot model (URDF +
  **SRDF**), the **planning scene** and its monitor, **planning pipelines** with request
  adapters, planner plugins, kinematics plugins, trajectory execution and controller
  managers, Servo, Hybrid Planning and the **Task Constructor**.
* Build a MoveIt configuration for the FR3 from `franka_description` and connect it to
  `ros2_control`.
* Use the C++ (and Python) APIs for planning with constraints, Cartesian paths, scene
  manipulation and attached objects.
* Write a **planner plugin** and a **kinematics plugin**, and benchmark them.

## Key ideas to own

* **Robot model = URDF + SRDF.** The SRDF adds **planning groups** (e.g. `fr3_arm`,
  `hand`), **end effectors**, **named states** (e.g. `ready`), **virtual joints**
  (robot-to-world attachment) and **disabled collision pairs** (adjacent links, pairs
  that never collide), which become the default allowed collision matrix.
* **Planning scene.** World geometry, the robot's current state, attached objects and the
  ACM. The *planning scene monitor* keeps it synchronised from `/joint_states`, TF,
  collision-object topics and optionally a sensor-driven octomap.
* **Planning pipeline.** A planner plugin (OMPL, Pilz industrial planner with
  PTP/LIN/CIRC, CHOMP, STOMP, and others) wrapped in **request/response adapters**:
  fixing start-state bounds, validating paths, and *time parameterisation* (TOTG by
  default; Ruckig smoothing is available). Pipelines can be chained, e.g. OMPL followed
  by CHOMP/STOMP refinement. Planning requests carry **goal constraints**
  (joint/position/orientation) and **path constraints**. OMPL plans path constraints by
  rejection sampling, or by projection when the constrained state space is enabled.
* **Kinematics plugins:** KDL (default), TRAC-IK, pick_ik, or yours. `kinematics.yaml`
  chooses the plugin per group and sets its timeout.
* **Execution.** The trajectory execution manager sends trajectories to controllers
  listed in `moveit_controllers.yaml` (usually a `FollowJointTrajectory` action on a JTC)
  and monitors them. `joint_limits.yaml` in the MoveIt config adds the **acceleration**
  limits that URDFs lack.
* **Cartesian paths.** `computeCartesianPath` interpolates the end effector in straight
  lines and solves IK at each step. Its **jump threshold** rejects joint-space jumps,
  the IK branch-flip problem from Modules 10 and 13.
* **Servo** gives real-time Cartesian or joint jogging with singularity and collision
  slow-down. **Hybrid Planning** runs a global planner plus a reactive local planner.
* **MoveIt Task Constructor (MTC)** builds a task as a tree of *stages* (generators,
  propagators and connectors) inside serial, parallel and alternative containers. It
  searches over stage solutions, so a pick-and-place becomes declarative.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [MoveIt 2 documentation](https://moveit.picknik.ai/main/index.html) (match the version to your distro; there is a [Humble version](https://moveit.picknik.ai/humble/)): Getting Started, "Your First C++ MoveIt Project", Planning Scene, Setup Assistant, Pick and Place with MTC, Concepts | Architecture and every API you need |
| [moveit/moveit2](https://github.com/moveit/moveit2) source: `moveit_ros/planning` and `moveit_planners/ompl` | How pipelines, adapters and the OMPL interface are implemented |
| [moveit/moveit_task_constructor](https://github.com/moveit/moveit_task_constructor) + M. Görner et al., "MoveIt! Task Constructor for Task-Level Motion Planning", ICRA 2019 | MTC concepts and API |

**Deeper**

| Resource | Take from it |
|---|---|
| [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2): the FR3 MoveIt config package | A reference config to compare yours with *after* you've built your own |
| [pick_ik](https://github.com/PickNikRobotics/pick_ik) | A modern kinematics plugin with cost terms |
| MoveIt benchmarking docs (`moveit_ros_benchmarks`) | Fair planner comparisons inside MoveIt |
| ROSCon talks on MoveIt 2, Servo and MTC ([roscon.ros.org](https://roscon.ros.org/), video archives) | Design rationale from the maintainers |

## Warm-ups

1. **Tour.** Run the MoveIt Getting Started demo (Panda). Plan and execute in RViz's
   MotionPlanning panel, then use `ros2 param list /move_group` to find the active
   pipeline, adapters and kinematics plugin.
2. **Setup Assistant on the FR3.** Generate a config from
   `franka_description/robots/fr3/fr3.urdf.xacro` (with the hand). Read every generated
   file and write one sentence per file on what it does.
3. **Scene API.** In C++, add boxes to the scene, attach a cylinder to the hand, then
   plan around obstacles with the object attached. Inspect the ACM before and after
   attaching.
4. **Cartesian lines.** Draw a 30 cm line with `computeCartesianPath`. Vary the jump
   threshold and step size and observe failures. Compare with your Module 13 pipeline on
   the same line.
5. **Pilz.** Plan LIN and CIRC motions (useful for cutting and welding-style paths) and
   compare their timing with OMPL + TOTG.
6. **Servo.** Jog the end effector with a keyboard or gamepad and drive it into a
   singularity. Observe the slow-down behaviour.

## Mini project: MoveIt, your way *(A: part 1; B: parts 2–3)*

**Part 1: `mamp_moveit_config`, hand-crafted *(A)*.** A MoveIt config for FR3 + hand
built from the xacro, integrated with your `mamp_bringup` (`ros2_control` mock,
controllers via `FollowJointTrajectory`). Include: acceleration and jerk limits in
`joint_limits.yaml` from Franka's specification; OMPL, Pilz and STOMP/CHOMP pipelines;
named states; an ACM generated by sampling. Benchmark kinematics plugins (KDL vs pick_ik
vs your Module 10 solver if you made it a plugin): success rate and time on 1,000
reachable poses.

**Part 2: your planner inside MoveIt *(B)*.** Wrap `mamp_planners` RRT-Connect (Module
16) as a `planning_interface::PlannerManager` plugin. Use MoveIt's planning scene for
validity checking (and, as a stretch, your own checker kept in sync with the scene).
Register it, select it from RViz, and benchmark it against OMPL's RRTConnect with
`moveit_ros_benchmarks` on your Module 16 scenes.

**Part 3: MTC pick-and-place *(B)*.** Pick a cube from a cluttered table and place it in
a bin. Use generated top and side grasps around the cube, Cartesian approach and retreat
stages, an **upright-orientation path constraint** while transporting (pretend it's a
cup), and alternatives for when a grasp fails. Execute on mock hardware, then in
simulation (Module 20).

**Proof of competence**

* The config launches cleanly, and planning succeeds on ≥ 99% of 100 random reachable
  goals in your scenes.
* Your plugin appears in RViz's planner list, with benchmark plots against OMPL.
* MTC: a recording showing every stage's solutions in the MTC RViz panel, and a success
  rate over 20 randomised cube placements.

**Stretch**

* Hybrid Planning with your MPC (Module 17) as the local planner.
* Servo with your QP-based differential IK (Module 10) in place of the default.
* A `moveit_py` notebook that runs the whole pick-and-place and collects benchmark data.

## Self-check

1. What does the SRDF add to the URDF, and why are those things kept separate?
2. What lives in the planning scene, and what keeps it up to date?
3. Name three planning request adapters and what each does.
4. Trace a "Plan & Execute" click from RViz down to `ros2_control`. Which actions and
   controllers are involved?
5. What problem does `computeCartesianPath`'s jump threshold address?
6. When would you choose Pilz, OMPL or STOMP?
7. Explain MTC's stage and container model with your pick-and-place as the example.

Next: [Module 20: Physics simulation](20-physics-simulation.md)
