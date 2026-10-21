# Module 17: Optimisation-based planning and optimal control

> **Part VI · Motion planning** · Est. 25–35 h · Prerequisites: 13, 15, 16 (and 11 for the dynamics parts) · Track B

## Why this matters / where it lives in this repo

Sampling-based planners give feasible but jerky paths, with no notion of task costs.
Optimisation-based planners produce **smooth, constraint-satisfying trajectories**, and
can add orientation constraints, clearance margins, torque limits and time. That is
exactly what Lab 2 step 4's plasma cutter needs ("orientation of the plasma cutter
matters to achieve a clean cut", "there are obstacles"). It is also the mathematical
bridge to **optimal control** and **MPC**, which run modern legged robots and
increasingly manipulators. Your maths background is a real advantage here.

## Learning objectives

* Formulate motion planning as a **nonlinear program**: decision variables, costs,
  equality and inequality constraints. Know which solver class fits.
* Use a numerical-optimisation toolkit: Gauss–Newton/LM, quasi-Newton, penalty and
  augmented-Lagrangian methods, SQP, interior-point methods, QP solvers; KKT conditions.
* Implement **CHOMP** and a **TrajOpt-style SQP** with signed-distance collision
  constraints, and understand **STOMP**, **KOMO**, **GPMP2** and **GCS**.
* Apply **direct transcription** (shooting, multiple shooting, collocation) and
  **iLQR/DDP** to robot dynamics.
* Close the loop with **MPC**, and know what makes it real-time.

## Key ideas to own

* **The NLP.** For waypoints $\xi = (q_0,\dots,q_N)$ (and possibly durations):
  $\min_\xi \sum_t \|q_{t+1} - 2q_t + q_{t-1}\|^2 + \text{task costs}$, subject to
  joint limits, boundary conditions, $d(q_t) \ge d_{\text{safe}}$, and task constraints
  (e.g. tool axis $\perp$ plate). Collision makes it non-convex, so solutions are
  **local** and initialisation matters.
* **Solver literacy.** First-order methods are cheap and slow. Gauss–Newton/LM exploit
  least-squares structure. **SQP** solves a sequence of QPs with trust regions or line
  searches. **Interior point** (IPOPT) handles inequalities with barriers. The KKT
  conditions characterise local optima; know how to read multipliers as constraint
  sensitivities. Sparsity (banded Hessians in time) is what makes all of these fast.
* **CHOMP** (covariant Hamiltonian optimisation). A functional gradient step
  preconditioned by $A^{-1}$, where $A$ is the finite-difference smoothness metric:
  $\xi \leftarrow \xi - \eta^{-1}A^{-1}\nabla U(\xi)$. This spreads each obstacle push
  smoothly along the whole trajectory. Obstacle cost comes from a precomputed signed
  distance field at body points, weighted by workspace speed.
* **STOMP** is gradient-free. It samples noisy trajectories, scores them, and combines
  them with path-integral-style weights. It handles non-differentiable costs.
* **TrajOpt** is SQP with an $\ell_1$ penalty on constraint violation and a trust
  region. Collision constraints are **linearised signed distances**
  ($d(q) \approx d(q_0) + \nabla d^\top(q - q_0)$, Module 15). It also has a
  continuous-time variant that checks the convex hull of consecutive link poses.
* **Direct methods for dynamics.** *Single shooting* optimises controls and simulates:
  few variables but ill-conditioned. *Multiple shooting* optimises states and controls
  with continuity constraints, which is robust. *Collocation* (trapezoidal,
  Hermite–Simpson) enforces dynamics at collocation points, sparse and accurate.
* **iLQR/DDP.** A backward Riccati-like pass on a local quadratic model, then a forward
  rollout. It costs $O(N)$ per iteration and gives feedback gains for free. Crocoddyl's
  FDDP handles infeasible initial guesses.
* **MPC.** Re-solve a finite-horizon problem every control period from the current state.
  Real-time feasibility comes from warm starts, sparse structure, early termination and
  the real-time iteration scheme (acados).
* **Hybrids win in practice.** Use sampling for global, topological decisions and
  optimisation for local refinement: MoveIt's planning adapters chain OMPL with
  CHOMP/STOMP-style refinement, and cuRobo runs many optimiser seeds in parallel on a GPU.
  **Graphs of Convex Sets** (GCS) instead decompose free space into convex regions and
  solve a mixed-integer convex problem with a tight relaxation, which gives near-global
  solutions.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| M. Kelly, "An Introduction to Trajectory Optimization: How to Do Your Own Direct Collocation", *SIAM Review* 2017, plus his video tutorial | Transcription methods, worked examples, practical tips |
| R. Tedrake, *Underactuated Robotics* ([free](https://underactuated.mit.edu/)): trajectory optimisation, LQR, MPC and motion-planning chapters, and the MIT 6.832 lectures on YouTube | Optimal control for robots, with Drake notebooks |
| N. Ratliff, M. Zucker, J. A. Bagnell & S. Srinivasa, "CHOMP: Gradient Optimization Techniques for Efficient Motion Planning", ICRA 2009 (and Zucker et al., IJRR 2013) | CHOMP |
| J. Schulman et al., "Motion Planning with Sequential Convex Optimization and Convex Collision Checking", IJRR 2014 | TrajOpt |
| J. Nocedal & S. Wright, *Numerical Optimization* (2nd ed., 2006), Ch. 2–4, 12, 17–19 | Line search, trust region, KKT, penalty, SQP, interior point |

**Deeper**

| Resource | Take from it |
|---|---|
| S. Boyd & L. Vandenberghe, *Convex Optimization* ([free](https://web.stanford.edu/~boyd/cvxbook/)) | Convexity, duality, QPs |
| M. Kalakrishnan et al., "STOMP: Stochastic Trajectory Optimization for Motion Planning", ICRA 2011 | Gradient-free trajectory optimisation |
| M. Mukadam, J. Dong, X. Yan, F. Dellaert & B. Boots, "Continuous-time Gaussian Process Motion Planning via Probabilistic Inference", IJRR 2018 (GPMP2) | Planning as inference on factor graphs |
| M. Toussaint, "Newton methods for k-order Markov Constrained Motion Problems" (arXiv 2014, KOMO) | Sparse Newton structure in time |
| Y. Tassa, T. Erez & E. Todorov, "Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization", IROS 2012 | iLQR/MPC in practice |
| C. Mastalli et al., "Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control", ICRA 2020 + [Crocoddyl](https://github.com/loco-3d/crocoddyl) | FDDP on Pinocchio |
| J. Rawlings, D. Mayne & M. Diehl, *Model Predictive Control: Theory, Computation, and Design* (2nd ed., [free PDF](https://sites.engineering.ucsb.edu/~jbraw/mpc/)) | MPC theory |
| T. Marcucci et al., "Motion Planning around Obstacles with Convex Optimization", *Science Robotics* 2023 ([arXiv:2205.04422](https://arxiv.org/abs/2205.04422)) | Graphs of Convex Sets |
| Tools: [CasADi](https://web.casadi.org/), [acados](https://docs.acados.org/), IPOPT, [OSQP](https://osqp.org/), [Drake](https://drake.mit.edu/), [cuRobo](https://curobo.org/) | What you'll actually build with |

## Warm-ups

1. **Optimiser bake-off.** On Rosenbrock and a constrained toy problem, implement
   gradient descent, Gauss–Newton, LM and an augmented-Lagrangian method. Compare them
   with IPOPT via CasADi. Plot iterates and convergence rates.
2. **Pendulum swing-up** by direct collocation (trapezoidal, then Hermite–Simpson),
   following Kelly. Plot the dynamics defects between collocation points.
3. **CHOMP in 2-D.** Plan for a point robot in an SDF world. Compare the covariant update
   with a plain Euclidean gradient step and explain the difference you see.
4. **iLQR** for a planar 2R arm reaching a target with torque limits (clamped or
   box-DDP).
5. **Linear MPC** for a double integrator with OSQP. Then nonlinear MPC on the 2R arm with
   acados or CasADi at 100 Hz.

## Mini project: `mamp_trajopt`, CHOMP + SQP + MPC, warm-started from your sampler *(B)*

**Scenes:** your Module 16 FR3 scenes plus a **"carry a full cup"** scenario (tool
z-axis within 5° of vertical throughout).

**Implement**
1. **CHOMP** (C++ or JAX) with obstacle cost from `mamp_collision`'s smooth distance
   (or a voxel SDF), covariant updates and workspace-speed weighting.
2. **TrajOpt-style SQP**: smoothness objective; joint-limit, collision
   ($d \ge d_{\text{safe}}$ via linearised signed distance) and orientation
   constraints; trust region; $\ell_1$ penalties; QP subproblems with OSQP. Also build a
   CasADi + IPOPT formulation as a baseline.
3. **Initialisation study:** straight-line in joint space, RRT-Connect output (Module
   16), and 32 random seeds in parallel (cuRobo-style, keep the best).
4. **Dynamics-aware refinement:** add torque limits via inverse dynamics (Module 11), or
   use Crocoddyl FDDP for a reaching task with torque bounds.
5. **MPC tracking** of the optimised trajectory in your Module 12 simulator at
   ≥ 100 Hz, with joint and torque limits, while applying a 2 s push disturbance.

**Proof of competence**

* A table over ≥ 4 scenes × 100 queries comparing (a) RRT-Connect + shortcut + TOPP-RA,
  (b) CHOMP, (c) SQP and (d) RRT-Connect → SQP. Columns: success, planning time, path
  length, integrated jerk, minimum clearance, execution time.
* A quantified **local-minimum** demonstration: optimisation from a straight-line start
  fails on the shelf scene, and warm-starting fixes it.
* Every reported success **independently certified** with your conservative motion
  validator (Module 15). Optimiser-reported feasibility doesn't count.
* An MPC disturbance-rejection plot with constraints respected throughout.

**Stretch**

* GCS in Drake (IRIS regions plus GCS) on a simplified shelf scene. Compare global vs
  local solution quality.
* A JAX GPU version of your SQP or CHOMP with 128 parallel seeds, timed against cuRobo.

## Self-check

1. Write the trajectory-optimisation NLP for a 7-DoF arm. What makes it non-convex?
2. State the KKT conditions. How do SQP and interior-point methods approach them
   differently?
3. Why does CHOMP precondition with $A^{-1}$, and what does that do to an obstacle
   gradient?
4. How does TrajOpt turn collision avoidance into constraints a QP can handle?
5. Single shooting vs multiple shooting vs collocation: give one strength and one weakness
   of each.
6. Why are iLQR and DDP linear in horizon length? What do you get besides the trajectory?
7. Why do hybrid sampling + optimisation planners dominate in practice?
8. What makes nonlinear MPC fast enough to run at 100 Hz–1 kHz?

Next: [Module 18: Constrained, kinodynamic and task-level planning](18-constrained-kinodynamic-task-planning.md)
