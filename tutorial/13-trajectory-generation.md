# Module 13: Trajectory generation and time parameterisation

> **Part V · Trajectories** · Est. 20–25 h · Prerequisites: 10, 12 (the `ros2_control` half) · Track A

## Why this matters / where it lives in this repo

Lab 2 is a trajectory-generation pipeline: Cartesian waypoints with segment times
([`cfg/traj_waypoints.yaml`](../Lab2/trajectory_demo/cfg/traj_waypoints.yaml)) → IK per
waypoint → a `JointTrajectory` of positions with `time_from_start` → JTC interpolation.
Its hints ("passing points closer together will often result in a cleaner final
trajectory", "be mindful of the robot's workspace", "add a timer") are all symptoms of
what this module teaches:

* Interpolating **in joint space** between two IK solutions does *not* produce a
  straight line in Cartesian space. Denser waypoints only approximate one.
* Positions-only points give **linear** JTC interpolation (Module 12): velocity jumps at
  every waypoint and infinite acceleration in theory.
* Fixed segment times ignore the robot's velocity and acceleration limits. Too short and
  you violate them; too long and you waste time. **Time parameterisation** solves this
  properly.

Read `create_traj` in
[`traj_demo.py`](../Lab2/trajectory_demo/trajectory_demo/traj_demo.py) as a reviewer:
trace by hand what `time_from_start` values it would produce for a three-waypoint
config, and check that this matches what the YAML's `time` field is meant to mean.

## Learning objectives

* Separate **path** (geometry, $q(s)$) from **timing** ($s(t)$).
* Generate point-to-point and multi-point trajectories: cubic and quintic polynomials,
  trapezoidal (LSPB) and double-S (jerk-limited) profiles, splines and B-splines,
  minimum-jerk.
* Generate **Cartesian** paths (straight lines with slerp, SE(3) geodesics and splines)
  and convert them to joint paths robustly.
* Compute **time-optimal parameterisations** under velocity, acceleration and torque
  limits (TOPP-RA), and **online jerk-limited** trajectories (Ruckig).
* Evaluate trajectories with quantitative smoothness and accuracy metrics.

## Key ideas to own

* **Polynomials.** A cubic matches position and velocity at both ends ($C^1$ across
  segments). A quintic also matches acceleration ($C^2$). Minimum-jerk motion between
  rest states is a specific quintic (Flash & Hogan), which also fits human reaching
  well.
* **Velocity profiles.** Trapezoidal (bounded $v$, $a$) gives infinite jerk at the
  corners. Double-S (7 segments, bounded $v$, $a$, $j$) is what industrial controllers
  and the Franka expect. Franka's controller enforces joint velocity, acceleration
  **and jerk** limits and aborts motions that violate them; the limits are listed in
  Franka's control-parameters documentation. The repo's `joint_limits.yaml` only carries
  velocity and effort.
* **Splines.** Cubic splines through via points ($C^2$, global support); B-splines
  (local control, convex-hull property, so limits can be enforced on control points);
  and on SE(3), cumulative B-splines or piecewise geodesics.
* **Cartesian vs joint interpolation.** A straight line in position with slerp in
  orientation is *not* the same as the SE(3) geodesic (a screw motion), and neither maps
  to a straight joint path. Cartesian paths need dense IK with continuity checks
  (Module 10) and can run into singularities mid-path.
* **Time-optimal path parameterisation (TOPP).** For a fixed path $q(s)$:
  $\dot q = q'(s)\,\dot s$ and $\ddot q = q'(s)\,\ddot s + q''(s)\,\dot s^2$. So joint
  velocity, acceleration *and torque* limits are **linear in $(\dot s^2, \ddot s)$**.
  TOPP-RA (Pham & Pham) exploits this with a backward pass of small LPs computing
  controllable sets, then a greedy forward pass. It is robust, fast and handles any
  linear constraint, including Cartesian speed caps. MoveIt's default (TOTG, Kunz &
  Stilman) is a related phase-plane method for velocity and acceleration limits.
* **Online generation.** Ruckig computes jerk-limited, time-optimal trajectories from
  *any* current state (position, velocity, acceleration) to a target state in
  microseconds. That is what you want for re-targeting during motion, servoing and
  smoothing.
* **Metrics.** Max and RMS Cartesian path deviation; total time against the TOPP lower
  bound; integrated squared jerk; peak joint velocity, acceleration and jerk as fractions
  of their limits; spectral arc length (SPARC) for perceived smoothness.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 9 + videos | Point-to-point, via points, time-optimal time scaling along a path (phase plane) |
| Siciliano et al., Ch. 4 | Joint- and operational-space trajectories, orientation interpolation |
| Q.-C. Pham & H. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis", IEEE T-RO 2018 + [toppra](https://github.com/hungpham2511/toppra) | TOPP-RA, with a usable Python library |
| L. Berscheid & T. Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States", RSS 2021 + [Ruckig](https://github.com/pantor/ruckig) | Online jerk-limited OTG |

**Deeper**

| Resource | Take from it |
|---|---|
| L. Biagiotti & C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (Springer, 2008) | Every profile and spline you'll ever need, with derivations |
| T. Kunz & M. Stilman, "Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity", RSS 2012 | The algorithm behind MoveIt's TOTG |
| T. Flash & N. Hogan, "The coordination of arm movements: an experimentally confirmed mathematical model", J. Neuroscience 1985 | Minimum jerk |
| [Franka control parameters](https://frankaemika.github.io/docs/control_parameters.html) (linked from Lab 1's `main.py`) | Joint and Cartesian velocity, acceleration and jerk limits for the real robot |
| [fontTools](https://github.com/fonttools/fonttools) | Glyph outlines for the mini project |

## Warm-ups

1. **Polynomials.** Derive cubic and quintic coefficients from boundary conditions.
   Implement them and plot position through jerk for a rest-to-rest move.
2. **Profiles.** Implement trapezoidal and double-S profiles from $(v_{\max}, a_{\max}, j_{\max})$.
   Verify the double-S total time against Ruckig for random rest-to-rest moves.
3. **Why denser is cleaner.** Take two Cartesian poses 30 cm apart, solve IK at both,
   and interpolate in joint space. Plot the end-effector path against the straight line.
   Repeat with 2, 4, 8, … intermediate IK waypoints and plot max deviation against
   waypoint count.
4. **Review `create_traj`.** Work out the `time_from_start` sequence it generates for
   segment times $(1, 1, 1)$. Is that what you'd expect? Write down what the code should
   compute if each `time` in the YAML is a segment duration.
5. **Phase plane.** Run TOPP-RA on a random joint-space path with FR3 velocity limits and
   Franka's acceleration limits. Plot the controllable set and the optimal $\dot s(s)$,
   and mark which constraint is active where.
6. **Re-targeting.** With Ruckig, change the target halfway through a motion and plot the
   seamless transition.

## Mini project: glyph-to-motion, robot handwriting done properly *(A)*

> Do this **after** submitting Lab 2, in your own package. It overlaps the "draw your
> initials" task, so check your course's policy before publishing it.

Make the FR3 write arbitrary text on a virtual whiteboard (a configurable plane pose).

**Pipeline**
1. Text and a TrueType font → glyph outlines (fontTools) → Bézier segments →
   **arc-length-parameterised** pen paths, with pen-up transitions (lift 2 cm).
2. Tool orientation: pen axis normal to the board, rotation about the pen axis free.
   Exploit that freedom (and the FR3's redundancy) to stay away from limits and
   singularities.
3. Dense IK along each stroke with your `mamp_ik` (Module 10), with continuity checks.
   Pen-up moves are planned in joint space.
4. **Time parameterisation** with TOPP-RA: joint velocity limits, acceleration limits,
   and a **Cartesian pen-speed cap** (≤ 0.2 m/s while writing, added as an extra linear
   constraint on $\dot s$). Finish with jerk-limited smoothing (Ruckig, or a B-spline fit
   with constraints on its control points).
5. Execute through `FollowJointTrajectory` with positions, velocities and accelerations
   (quintic JTC interpolation) on `mamp_bringup`. Later, in simulation (Module 20).
6. Analyse with `mamp_analyze` (Module 04).

**Baseline for comparison:** a Lab-2-style approach with sparse Cartesian waypoints,
fixed segment times and positions-only points.

**Proof of competence**

* Writes "MaMP" (or your name) with **max deviation < 1 mm** from the glyph path
  (measured from TF on the executed reference), **zero** joint velocity, acceleration or
  jerk limit violations, and total time within 15% of the TOPP-RA lower bound.
* A comparison table and figures against the baseline: deviation, time, integrated
  jerk, peak limit ratios.
* No IK branch flips (the automatic detector from Module 04 passes).

**Stretch**

* Write on a **cylinder** (the pen normal follows the surface).
* Replace pen-down strokes with SE(3) cumulative B-splines (Module 05) and compare
  smoothness.
* Human-like minimum-jerk timing against time-optimal timing: run a blind survey of
  friends rating which "looks more natural".

## Self-check

1. Define path, trajectory and time scaling. Why separate them?
2. Why does joint-space interpolation between two IK solutions produce a curved
   Cartesian path, and why does waypoint density help?
3. Which boundary conditions do cubic and quintic segments satisfy, and what continuity
   do they give across waypoints?
4. Why are joint velocity, acceleration and torque limits linear in $(\dot s^2, \ddot s)$
   along a fixed path?
5. When would you use Ruckig instead of TOPP-RA, and vice versa?
6. What happens on a real Franka if a commanded trajectory violates the jerk limits?
7. List three problems with sending positions-only waypoints at fixed times to the JTC.

Next: [Module 14: Configuration space and graph search](14-configuration-space-and-search.md)
