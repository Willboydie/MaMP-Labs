# Module 10: Inverse kinematics and redundancy resolution

> **Part III · Kinematics** · Est. 20–30 h · Prerequisites: 09 · Track A

## Why this matters / where it lives in this repo

Lab 2 converts Cartesian waypoints to joint waypoints with
`PyKDL.ChainIkSolverPos_LMA` (Levenberg–Marquardt), seeding each solve with the previous
solution. That one design decision decides whether the trajectory is smooth or jumps
between IK branches. Read `get_joint_pos` and `create_traj` in
[`traj_demo.py`](../Lab2/trajectory_demo/trajectory_demo/traj_demo.py) critically:

* When `target_orientation is None`, the goal is `PyKDL.Frame(pos)`. **What orientation
  does that frame contain**, and how does LMA's error-weighting vector treat it? Is the
  starter's "position-only" IK really position-only? Find out from the KDL source.
* LMA **does not enforce joint limits**. Lab 2's mock robot also starts at $q = 0$, which
  is already outside the FR3's joint-4 and joint-6 limits (Module 08).
* The FR3 has 7 joints for a 6-D task, so every reachable pose has a 1-D family of
  solutions. Which one LMA returns depends only on the seed.

Lab 2 step 3 adds orientation targets, and step 4 (the plasma cutter) is about
constrained, redundant IK along a path.

## Learning objectives

* Classify IK problems: closed-form vs numerical, 6-DoF vs redundant, position-only vs
  full-pose.
* Implement Newton–Raphson, Jacobian-transpose, damped least squares and adaptive
  Levenberg–Marquardt IK on SE(3), with correct error metrics and stopping criteria.
* Resolve **redundancy** with null-space projection and task priorities, and handle joint
  limits properly (clamping vs QP constraints).
* Formulate IK as an optimisation problem (QP-based differential IK, NLP-based IK) and
  know the main libraries.
* Benchmark IK solvers fairly and design path IK that avoids branch flips.

## Key ideas to own

* **Solution structure.** A generic 6R arm has up to 16 solutions per pose (at most 8
  for common industrial geometries). A 7-DoF arm has a 1-D self-motion manifold per pose
  (parameterised by the elbow swivel, or for the FR3 conveniently by $q_7$), cut into
  pieces by joint limits.
* **Closed form.** Pieper's criterion: three consecutive axes intersecting at a point (a
  spherical wrist) decouples position from orientation. The FR3's wrist is offset
  ($a = 0.088$ m), so standard decoupling fails. A geometric analytical solver
  parameterised by $q_7$ exists (He & Liu, 2021). IKFast generates closed-form solvers for
  many arms.
* **Newton on SE(3).** Error $e = \log\!\big(T(q)^{-1}T_d\big)^\vee$ (body frame) or its
  space-frame equivalent. Update $\Delta q = J_b^{+}e$, with step limiting or a line
  search. Converges quadratically near a regular solution and misbehaves near
  singularities.
* **Damped least squares / Levenberg–Marquardt.**
  $\Delta q = J^\top(JJ^\top+\lambda^2 I)^{-1}e$. $\lambda$ trades accuracy for
  robustness near singularities. Adaptive schemes (e.g. Sugihara's $\lambda = e^\top W_e e
  + \bar\lambda$) make LM "solvability-unconcerned": it returns the closest feasible pose
  when the target is unreachable. A weighting $W_e$ balances metres against radians, and
  that is exactly KDL LMA's `L` vector.
* **Redundancy.** $\dot q = J^{+}\dot x + (I - J^{+}J)\,\dot q_0$ with
  $\dot q_0 = k\nabla H(q)$, where $H$ is a secondary objective (joint-range centring,
  manipulability, obstacle distance). Strict task priorities use nested projectors
  (Nakamura; Siciliano & Slotine); Chiaverini's singularity-robust variant damps the
  lower-priority task.
* **Joint limits.** Clamping after each step breaks convergence guarantees. Better: a
  weighted or gradient-projection approach, or a **QP**:
  $\min_{\dot q}\|J\dot q - v_d\|^2 + \epsilon\|\dot q\|^2$ subject to
  $q_{\min}\le q+\dot q\,\Delta t\le q_{\max}$ and $|\dot q|\le\dot q_{\max}$. Pink and
  mink do this; MoveIt Servo and Drake's differential IK are of this family.
* **Global strategies.** Random restarts, as TRAC-IK does (it races a KDL-style Newton
  solver and an SQP solver in parallel threads); nonlinear programming with constraints
  (IPOPT/SLSQP); even global optimality via mixed-integer convex relaxations (Dai et al.,
  2017).
* **Path IK.** Seeding each waypoint with the previous solution keeps you on one branch
  *as long as* steps are small and you stay away from singularities and limits. Detect
  jumps (large $\|\Delta q\|$ for a small Cartesian step) and treat them as errors.
  Modules 13 and 18 go further.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 6 + videos ([free](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)) | Analytic and numerical IK, Newton on SE(3) |
| Siciliano et al., Ch. 3 (inverse kinematics algorithms, redundancy) | Jacobian inverse/transpose algorithms, closed-loop IK, null-space objectives |
| S. R. Buss, ["Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods"](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf) (2004) | The clearest short comparison of the methods |
| KDL source: [`chainiksolverpos_lma.cpp`](https://github.com/orocos/orocos_kinematics_dynamics) (in `orocos_kdl/src/`) | The exact algorithm Lab 2 runs: error, weights, damping, return codes |

**Deeper**

| Resource | Take from it |
|---|---|
| T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg–Marquardt Method", IEEE T-RO 2011 | Adaptive damping that works on unreachable targets |
| P. Beeson & B. Ames, "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics", Humanoids 2015 | Why random restarts + SQP beat plain Newton on joint-limited arms |
| Y. He & S. Liu, "Analytical Inverse Kinematics for Franka Emika Panda – a Geometrical Solver for 7-DOF Manipulators with Unconventional Design", ICCMA 2021; implementations [frankik](https://pypi.org/project/frankik/) and [CoMMALab/frantik](https://github.com/CoMMALab/frantik) | All solutions for a given $q_7$ |
| S. Chiaverini, "Singularity-Robust Task-Priority Redundancy Resolution for Real-Time Kinematic Control of Robot Manipulators", IEEE TRA 1997 | Task priority done right |
| R. Diankov, *Automated Construction of Robotic Manipulation Programs* (PhD thesis, CMU, 2010) | IKFast |
| Tedrake, *Robotic Manipulation*, differential IK and IK-as-optimisation chapters ([manipulation.mit.edu](https://manipulation.mit.edu/)) | QP-based differential IK in Drake |
| [Pink](https://github.com/stephane-caron/pink), [mink](https://github.com/kevinzakka/mink), [qpsolvers](https://github.com/qpsolvers/qpsolvers), [pick_ik](https://github.com/PickNikRobotics/pick_ik) | Modern task-based and QP IK libraries |

## Warm-ups

1. **Read the solver.** From KDL's LMA source, write down: the error definition, the
   default weights, the damping update, the stopping criteria and every return code.
   Does it ever look at joint limits?
2. **Is it position-only?** Using what you found, predict how the starter's
   `PyKDL.Frame(pos)` target and LMA's weights interact. Test your prediction by solving
   the same position with different weight vectors and comparing the resulting
   orientations.
3. **Basins of attraction.** For a planar 3R arm, run pseudo-inverse Newton and DLS from
   a 200×200 grid of seeds (two joints varied) to a fixed target. Colour each seed by the
   solution it reaches. Explain the fractal-looking boundaries.
4. **Self-motion.** Hold the FR3's flange pose fixed and move along the null space
   ($\dot q = (I - J^+J)\dot q_0$) while visualising in RViz. Then enumerate solutions
   with an analytic solver (frankik/frantik) by sweeping $q_7$, and overlay them.
5. **Seed matters.** Solve the same 100 targets from $q = 0$ and from Franka's usual
   "ready" pose $[0, -\pi/4, 0, -3\pi/4, 0, \pi/2, \pi/4]$ (it appears as the
   `initial_position`s in
   [`franka_arm.ros2_control.xacro`](../franka_description/robots/common/franka_arm.ros2_control.xacro)).
   Count the joint-limit violations in each case.

## Mini project: `mamp_ik`, your own IK library, benchmarked *(A: solvers 1–3 + benchmark; B: the rest)*

Write a ROS-free C++ library (Python bindings, with a ROS 2 service wrapper) containing:

1. **Newton–Raphson** with pseudo-inverse and backtracking line search on the SE(3) log
   error.
2. **Adaptive LM** (Sugihara-style) with a configurable position/orientation weighting.
3. **Joint limits** handled three ways: clamping, gradient projection, and a QP-based
   differential IK (OSQP).
4. **Null-space objectives:** joint-range centring, manipulability, distance to seed.
5. A **random-restart, time-budgeted** wrapper that runs two solver flavours in parallel
   threads, first to succeed wins.
6. **Analytic FR3 IK**, implemented from He & Liu's paper and verified against frankik,
   returning all solutions for a given $q_7$.
7. A `ComputeIK` service in your `mamp_interfaces` package.

**Benchmark harness.** Generate 10,000 reachable targets by FK of random within-limit
configurations. Use three seed regimes: random, near-solution (perturbed), and
Lab-2-style $q = 0$. Measure success (position error < 1e-5 m and orientation error
< 1e-4 rad, **within joint limits**), time distribution, distance from seed, and the
manipulability of the solution. Compare against KDL LMA, KDL `ChainIkSolverPos_NR_JL`,
TRAC-IK, and Pink or mink.

**Path IK test.** Follow a 1,000-point line, circle and figure-of-eight in Cartesian
space with fixed tool orientation, seeding from the previous solution. Report the
maximum per-step joint jump. Then use analytic IK with a continuity-preserving choice of
$q_7$ and compare.

**Proof of competence**

* A benchmark table and CDF plots. Your best numerical solver beats KDL LMA's
  within-limits success rate from random seeds by a clear margin, at a comparable median
  time.
* The analytic solver reaches 100% success on reachable poses, at microsecond-scale cost.
* Zero branch flips on the path tests for your best method, with a documented example
  where seed-chained LMA flips.

**Stretch**

* Package your solver as a MoveIt 2 kinematics plugin (finish it in Module 19).
* Train a small network to predict good seeds from target poses and measure the speed-up.
* Global IK for a small problem with Drake's mixed-integer formulation, to certify that a
  pose is unreachable within joint limits.

## Self-check

1. How many IK solutions does a generic reachable pose have for a 6R arm, and what does
   the solution set look like for the FR3?
2. Why does Pieper's criterion not apply to the FR3? What does the analytic solver use
   instead?
3. Derive the DLS update from a regularised least-squares problem. What does $\lambda$
   trade off?
4. Write the null-space projector. Why isn't the secondary objective guaranteed to be
   achieved?
5. Why seed each waypoint with the previous solution? Name two ways it still fails.
6. In Lab 2's starter, what orientation is requested when `target_orientation` is
   `None`, and what effect does that have?
7. Compare clamping, gradient projection and QP constraints for joint limits.

Next: [Module 11: Rigid-body dynamics](11-rigid-body-dynamics.md)
