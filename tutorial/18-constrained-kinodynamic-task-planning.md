# Module 18: Constrained, kinodynamic and task-level planning

> **Part VI · Motion planning** · Est. 20–30 h · Prerequisites: 16, 17 · Track B

## Why this matters / where it lives in this repo

Real tasks are rarely "go from A to B avoiding obstacles". Lab 2 step 4's plasma cutter
is a textbook example of everything in this module:
* the torch tip must **follow a given path** on the plate (a Cartesian path constraint);
* the tool axis must stay **normal to the plate** while rotation about it is free (a
  manifold constraint with useful redundancy);
* the cut needs a **constant speed** (a timing constraint);
* there are **obstacles** and **transitions** between shapes (planning between
  constrained segments);
* and the **order** of cuts, and whether to reposition the plate, is a task-level
  decision.

Step 4 asks you to *describe* strategies. This module gives you the tools to *build*
them. The same ideas cover mobile bases (non-holonomic constraints), systems with
dynamics (kinodynamic planning) and multi-step manipulation (task and motion planning).

## Learning objectives

* Plan on **implicitly defined constraint manifolds** $F(q) = 0$ with projection,
  tangent-space and atlas methods, and use **Task Space Regions** for pose constraints
  with tolerances.
* Solve **Cartesian path-following** with redundancy as a graph search (ladder graphs,
  Descartes-style), and know the industrial tooling (Tesseract).
* Formulate and solve **kinodynamic** problems (state + control, differential
  constraints): kinodynamic RRT, SST, LQR-based steering.
* Handle **non-holonomic** systems: controllability, Dubins and Reeds–Shepp curves,
  differential flatness, state lattices and Hybrid A*.
* Explain **task and motion planning (TAMP)** and when you need it.

## Key ideas to own

* **Constraint manifolds have measure zero.** Uniform sampling *never* hits
  $\{q : F(q) = 0\}$, so samplers must generate constraint-satisfying states. Options:
  * **Projection:** Newton steps $q \leftarrow q - J_F^{+}F(q)$ (CBiRRT, OMPL's
    `ProjectedStateSpace`).
  * **Tangent space:** sample in $\ker J_F$, then project.
  * **Atlas:** piece together local charts (AtlasRRT, OMPL's `AtlasStateSpace`).

  Edges must be checked *on* the manifold too.
* **Task Space Regions** (Berenson et al.) describe a set of end-effector poses: a frame
  plus bounds on each of the 6 coordinates. "Tool z-axis normal to the plate, any
  rotation about it, tip on the contour ± 0.5 mm" is one TSR per path point. Goal TSRs
  also make grasping much easier.
* **Cartesian path following with redundancy.** Discretise the free DoFs at each path
  point (rotation about the tool axis × IK branches × $q_7$ samples for the FR3). Each
  point becomes a "rung" of candidate joint configurations, and consecutive rungs are
  connected by feasible edges (joint velocity at the required speed, collision-free). A
  DAG shortest path then gives the globally best joint path *for that discretisation*.
  This is the ladder-graph idea behind ROS-Industrial's Descartes, used for welding,
  painting and cutting, and the basis of a strong answer to Lab 2 step 4.
* **Kinodynamic planning.** State $x = (q, \dot q)$, controls $u$, $\dot x = f(x,u)$,
  with bounds on both. The *steering problem* (connect two states exactly) is hard in
  general, so kinodynamic RRT extends by random or best-of-$k$ control rollouts. **SST**
  gets asymptotic near-optimality *without* steering. LQR-based distance and steering
  (LQR-RRT\*, kinodynamic RRT\*) works for linearisable systems.
* **Non-holonomic constraints.** These are velocity constraints that don't integrate to
  position constraints: a car can't slide sideways but can still reach any pose
  (parallel parking). Controllability follows from Lie brackets (Chow's theorem).
  Dubins curves (forward only) and Reeds–Shepp curves (with reverse) are the optimal
  paths for the bounded-curvature car and make perfect steering functions. Differentially
  flat systems (cars, quadrotors) turn trajectory design into curve design (e.g.
  minimum-snap polynomials). State lattices and Hybrid A* bring search to these systems.
* **TAMP.** Choosing *which* actions to take (pick A, place on B, regrasp…) interacts
  with *whether* they are geometrically feasible. Approaches: sampling-based
  symbolic–geometric search (PDDLStream), optimisation-based Logic-Geometric Programming,
  and pragmatic pipelines of stages (MoveIt Task Constructor, Module 19).
* **Multi-robot and dual-arm planning** live in a composite C-space. They use
  decoupled/prioritised planning (fast, incomplete), coupled planning (complete, slow) or
  hybrids (dRRT). Closed chains, such as two arms holding one object, are constraint
  manifolds again.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Z. Kingston, M. Moll & L. Kavraki, "Sampling-Based Methods for Motion Planning with Constraints", *Annu. Rev. Control Robot. Auton. Syst.* 2018 | Survey of projection, tangent-space and atlas methods |
| D. Berenson, S. Srinivasa & J. Kuffner, "Task Space Regions: A Framework for Pose-Constrained Manipulation Planning", IJRR 2011 | TSRs and CBiRRT |
| [OMPL constrained planning tutorial](https://ompl.kavrakilab.org/constrainedPlanning.html) | Practical constrained planning |
| LaValle, *Planning Algorithms* ([free](https://lavalle.pl/planning/)), Ch. 13–15 | Differential models, kinodynamic sampling-based planning, Dubins/Reeds–Shepp, controllability |
| Lynch & Park, *Modern Robotics*, Ch. 13 | Non-holonomic wheeled robots, controllability, motion planning for cars |

**Deeper**

| Resource | Take from it |
|---|---|
| S. LaValle & J. Kuffner, "Randomized Kinodynamic Planning", IJRR 2001 | Kinodynamic RRT |
| Y. Li, Z. Littlefield & K. Bekris, "Asymptotically Optimal Sampling-Based Kinodynamic Planning", IJRR 2016 | SST without a steering function |
| D. Dolgov, S. Thrun, M. Montemerlo & J. Diebel, "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments", IJRR 2010 | Hybrid A* |
| M. Pivtoraiko, R. Knepper & A. Kelly, "Differentially Constrained Mobile Robot Motion Planning in State Lattices", J. Field Robotics 2009 | State lattices |
| C. Garrett et al., "Integrated Task and Motion Planning", *Annu. Rev. Control Robot. Auton. Syst.* 2021 + [PDDLStream](https://github.com/caelan/pddlstream) | The TAMP landscape and a reference implementation |
| [Tesseract](https://github.com/tesseract-robotics/tesseract) and [descartes_light](https://github.com/swri-robotics/descartes_light) | Industrial process planning: Cartesian path graphs with optimisation refinement |
| [Nav2 documentation](https://docs.nav2.org/) | How mobile-robot planning (Smac Hybrid-A\*, state lattice) is done in ROS 2 |
| Tedrake, *Robotic Manipulation* ([manipulation.mit.edu](https://manipulation.mit.edu/)), task-level planning chapter | Task planning for manipulation |

## Warm-ups

1. **Projection.** For the FR3, implement a projection operator onto "tool z-axis
   vertical". Sample 10⁴ random configurations, project them, and measure the success
   rate and the distribution of residuals.
2. **Constrained RRT-Connect.** Using OMPL's `ProjectedStateSpace` (or your own
   projection in `mamp_planners`), plan a cup-carrying motion between two shelves while
   keeping the cup upright.
3. **Dubins and Reeds–Shepp.** Implement both and visualise all candidate words for
   random start/goal pairs. Verify optimality against OMPL's implementations.
4. **Hybrid A\*** for a car in a parking lot with Reeds–Shepp analytic expansion. Animate
   the result.
5. **Kinodynamic RRT vs SST** for a 2-D double integrator with velocity and acceleration
   bounds: plot cost against time.

## Mini project: the Lab 2 plasma cutter, built for real *(B)*

> Do this after submitting Lab 2, so your step 4 essay stays your own reasoning. Then
> compare what you wrote with what actually worked.

**Scene:** the FR3 (or your Module 07 dual-arm robot), a 40×30 cm steel plate on a
table, two clamps and a fixture post intruding into the workspace, and a torch tool
frame (add a tool link via xacro: a 15 cm torch with its TCP at the tip).

**Task:** cut a set of closed contours (a circle, a star, the outline of your initials)
with:
* the torch tip on the contour at 2 mm standoff;
* the torch axis within **±2°** of the plate normal, with rotation about the axis free;
* a constant **cut speed** of 20 mm/s, with lead-in and lead-out arcs;
* collision-free transitions between contours (torch lifted 5 cm).

**Methods to implement and compare**

1. **Ladder graph (Descartes-style).** At each path point, sample the free rotation
   (e.g. 36 values) × analytic FR3 IK solutions over sampled $q_7$ (Module 10). Prune
   colliding and out-of-limit nodes. Connect rungs only where the required joint speed at
   20 mm/s is within limits. Find the DAG shortest path minimising joint motion.
2. **Constrained sampling-based transitions** between contours (Module 16 planners plus
   projection), with the torch kept pointing down.
3. **Optimisation refinement** (Module 17): SQP over the whole program, with path,
   orientation and speed constraints, warm-started from method 1.
4. **Timing:** exact constant speed during cuts (verify feasibility against joint
   velocity and acceleration limits), time-optimal transitions (TOPP-RA).

Execute in `mamp_bringup` (mock) and in simulation (Module 20). Analyse with
`mamp_analyze`.

**Proof of competence**

* Every contour cut with tip deviation **< 0.5 mm**, axis error **< 2°** and speed
  variation **< 5%**, with all joint limits respected and **zero** collisions (certified
  with your conservative checker).
* Transitions found for 100% of contour pairs.
* A comparison of methods 1–3: computation time, joint-path length, smoothness, and
  which constraints each struggles with.
* A one-page reflection: what your Lab 2 step 4 essay got right and what it missed.

**Stretch**

* **Dual-arm:** one arm repositions the plate (a closed-chain constraint while both
  grasp it) so the cutting arm can reach a contour that is otherwise unreachable.
* **TAMP:** decide the cut order and whether and where to reposition the plate with
  PDDLStream, using your planners as samplers and feasibility checkers.
* **Mobile base:** put the robot on a differential-drive base and plan base placement
  plus arm motion for a plate too large to reach from one pose.

## Self-check

1. Why do uniform samplers fail on constraint manifolds? Compare projection, tangent-space
   and atlas methods.
2. Write a TSR for "torch axis normal to the plate, free rotation about it, tip on the
   path ± 0.5 mm".
3. Why does Cartesian path following with redundancy become a shortest-path problem on a
   DAG? What is lost by discretising?
4. What is the steering problem, and how does SST avoid needing one?
5. Non-holonomic vs underactuated: define both and give an example of each. Why can a car
   parallel park?
6. When are Dubins curves optimal, and what do Reeds–Shepp curves add?
7. What does TAMP add beyond motion planning, and why is it computationally hard?

Next: [Module 19: MoveIt 2](19-moveit2.md)
