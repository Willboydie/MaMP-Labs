# Module 20: Physics simulation (Gazebo, MuJoCo, Drake, Isaac)

> **Part VII · Frameworks and simulation** · Est. 20–30 h · Prerequisites: 11, 12 · Track B

## Why this matters / where it lives in this repo

Lab 2 runs on `mock_components/GenericSystem`. It's ideal for plumbing, but it has no
dynamics, no contact and no tracking error. `franka_description` already anticipates
real simulation:
* `gazebo` and `gazebo_effort` xacro arguments in
  [`fr3.urdf.xacro`](../franka_description/robots/fr3/fr3.urdf.xacro);
* `<gazebo>` friction and transmission tags in
  [`franka_robot.xacro`](../franka_description/robots/common/franka_robot.xacro);
* an Ignition/Gazebo `ros2_control` plugin in
  [`franka_arm.ros2_control.xacro`](../franka_description/robots/common/franka_arm.ros2_control.xacro);
* `gazebo_ros gazebo_model_path` exported in
  [`package.xml`](../franka_description/package.xml).

This module teaches what simulators actually compute, how to connect them to ROS 2
through `ros2_control`, and how to reason about the gap between simulation and reality.

## Learning objectives

* Explain a simulator's step: dynamics integration, collision detection, contact
  resolution, actuators and sensors.
* Compare the main **contact models**: complementarity/LCP, penalty, MuJoCo's convex
  soft-constraint formulation, and Drake's hydroelastic contact.
* Run the FR3 in **Gazebo** and **MuJoCo** through `ros2_control`, with simulated time,
  and know the version pairings.
* Know when to reach for **Drake**, **Isaac Sim/Isaac Lab** or **PyBullet** instead.
* Quantify and reduce the **sim-to-real gap**: identification, actuator models,
  latency, domain randomisation.

## Key ideas to own

* **One step of simulation.** Compute forward dynamics (Module 11), detect contacts
  (Module 15), solve for contact impulses or forces subject to non-penetration and
  friction, integrate, update sensors. Most "physics bugs" are contact-solver or
  timestep artefacts.
* **Contact models.**
  * *Hard* (velocity-level complementarity, Stewart–Trinkle/Anitescu): an LCP or NCP each
    step, with a friction cone often linearised to a pyramid.
  * *Penalty* (spring–damper): simple, but stiff, needing small timesteps.
  * *MuJoCo:* a convex optimisation with soft constraints. Fast, stable and
    differentiable-ish, with tunable softness (`solref`/`solimp`).
  * *Drake hydroelastic:* pressure fields over contact patches, which gives better
    contact forces for grasping.
* **Gazebo** (now "gz sim"; formerly Ignition) uses SDF worlds and plugin physics engines
  (DART by default, Bullet, TPE), and connects through `gz_ros2_control` and
  `ros_gz_bridge`. **Pairings:** Humble ↔ Fortress, Jazzy ↔ Harmonic, Lyrical ↔ Jetty.
  Gazebo Classic is end-of-life. The Franka xacro still references
  `franka_ign_ros2_control` naming from the Ignition era.
* **MuJoCo** uses MJCF with explicit actuators and contact parameters, has excellent
  Python bindings and a viewer, and has GPU variants (MJX on JAX, plus newer GPU
  backends) for batched simulation. `mujoco_ros2_control` (ros-controls) exposes a
  `ros2_control` system interface (binaries for Jazzy and later). MuJoCo Menagerie
  has a tuned `franka_fr3`.
* **Drake** combines model-based design, optimisation and accurate contact (good for
  research on planning and control). **Isaac Sim / Isaac Lab** offers GPU PhysX at scale
  with photorealistic sensors, mainly for RL (Module 21); it needs an NVIDIA RTX GPU.
  **PyBullet** is easy and legacy.
* **Simulated actuators.** Position-controlled joints in simulators are often PD loops
  with arbitrary gains, or worse, kinematic overrides. Effort control with your own
  controllers (Module 12) is far more honest. Match command interfaces to what the real
  robot offers.
* **Time.** Simulators publish `/clock`; set `use_sim_time:=true` on **every** node,
  including RViz, `robot_state_publisher` and MoveIt. Watch the real-time factor, and
  run step-by-step for deterministic tests.
* **Sim-to-real.** Gaps come from friction, joint elasticity, actuator bandwidth,
  latency, sensor noise and contact. Mitigations: identification (Module 11), actuator
  and latency models, domain randomisation, and conservative controller design. Always
  report results with the simulator's configuration.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [MuJoCo documentation](https://mujoco.readthedocs.io/): Overview, **Computation** chapter (the contact model), Modelling, Python bindings and tutorial notebooks | How a modern simulator works, explained by its authors |
| E. Todorov, T. Erez & Y. Tassa, "MuJoCo: A physics engine for model-based control", IROS 2012 | The original design |
| [Gazebo documentation](https://gazebosim.org/docs) (select your version) and [gz_ros2_control](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html) | Worlds, plugins and the ROS 2 bridge |
| [mujoco_ros2_control](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html) + [MuJoCo Menagerie `franka_fr3`](https://github.com/google-deepmind/mujoco_menagerie) | MuJoCo behind `ros2_control` |
| Tedrake, *Robotic Manipulation* ([manipulation.mit.edu](https://manipulation.mit.edu/)), the simulation and contact chapters, with [Drake](https://drake.mit.edu/) notebooks | Contact modelling for manipulation, hands-on |

**Deeper**

| Resource | Take from it |
|---|---|
| T. Erez, Y. Tassa & E. Todorov, "Simulation Tools for Model-Based Robotics: Comparison of Bullet, Havok, MuJoCo, ODE and PhysX", ICRA 2015 | How to compare simulators rigorously |
| D. Stewart & J. Trinkle, "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Inelastic Collisions and Coulomb Friction", IJNME 1996 | The complementarity formulation |
| R. Elandt et al., "A pressure field model for fast, robust approximation of net contact force and moment between nominally rigid objects", IROS 2019 | Hydroelastic contact (Drake) |
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) | GPU-parallel simulation for learning |
| J. Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World", IROS 2017; X. B. Peng et al., "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization", ICRA 2018 | Domain and dynamics randomisation |

## Warm-ups

1. **Menagerie vs `franka_description`.** Load MuJoCo Menagerie's `franka_fr3` in the
   MuJoCo viewer. Compare link masses and inertias with `inertials.yaml` and list the
   differences. Read its actuator definitions: what are the command and control ranges?
2. **Gazebo bring-up.** Spawn the FR3 in Gazebo (Fortress on Humble, Harmonic on Jazzy)
   with `gz_ros2_control` and a JTC. Send Lab 2's example trajectory and plot tracking
   error, which is non-zero for the first time.
3. **Drop test.** Drop the same box onto a table in MuJoCo, Gazebo and PyBullet with
   "the same" parameters. Compare bounce and settling time, then vary the timestep until
   each one goes unstable.
4. **Energy check.** Simulate a frictionless FR3 released from rest in MuJoCo and in your
   own simulator (Module 11). Compare energy drift and joint trajectories over 10 s.
5. **Sim time.** Pause the simulator and confirm which of your nodes freeze and which keep
   running. Fix any that run on wall time but shouldn't.

## Mini project: one controller, four "hardwares" *(B)*

Extend `mamp_bringup` with `hardware:=mock|mamp_sim|gazebo|mujoco`. Keep the same FR3
description, the same controller YAML (JTC plus your Module 12 controllers) and the same
tasks: Module 13 handwriting, Module 18 cutting and Module 19 pick-and-place.

**Experiments**
1. **Tracking:** figure-of-eight and glyph trajectories with position and effort command
   interfaces. Report RMS and maximum error per hardware.
2. **Contact:** push a 0.5 kg block 10 cm with the end effector, and grasp and lift a
   cube with the Franka hand. Compare block displacement and grasp success.
3. **Timing and determinism:** real-time factor, and whether two identical runs are
   bit-identical.
4. **Identification:** excite each joint with a chirp and identify damping and friction
   in each simulator. Compare with the configured values.
5. **Domain randomisation:** randomise masses (±20%), friction (×0.5–2), sensor latency
   (0–20 ms) and controller gains in MuJoCo. Measure pick-and-place success over 200
   episodes.

**Proof of competence**

* A comparison report with plots, and a **parameter alignment table** listing what you
  had to change to make the simulators agree, and what you couldn't align and why.
* Discrepancies explained in terms of contact models, actuator models and integrators,
  not hand-waving.
* Determinism results, and a domain-randomisation success curve.

**Stretch**

* Batched simulation with MJX (or another GPU backend): 1,000 parallel pick attempts.
* Port the pick task to Isaac Lab and compare its throughput.
* Add a penalty contact model with a friction cone to your own simulator, and reproduce
  the block-push experiment against MuJoCo.

## Self-check

1. What does a physics engine compute in one step, and in what order?
2. Compare complementarity, penalty, MuJoCo-style convex and hydroelastic contact.
3. Why do simulated position-controlled joints often behave unlike real ones?
4. What makes a simulation go unstable, and what are your first three fixes?
5. Which Gazebo version pairs with Humble, Jazzy and Lyrical?
6. Name three sources of sim-to-real gap for the FR3 and a mitigation for each.
7. What breaks if one node in your launch forgets `use_sim_time:=true`?

Next: [Module 21: Robot learning for motion](21-robot-learning-for-motion.md)
