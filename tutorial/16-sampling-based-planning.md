# Module 16: Sampling-based motion planning

> **Part VI · Motion planning** · Est. 25–35 h · Prerequisites: 14 (and 15 for the full project) · Track A

## Why this matters / where it lives in this repo

Sampling-based planners are how real 7-DoF arms find collision-free paths. They are the
default in MoveIt (OMPL's RRT-Connect), the backbone of almost every manipulation
pipeline, and the core of a "Motion Planning" module. They answer Lab 2 step 4's "there
are obstacles in the robot's workspace" in a way that waypoint IK can't. This is also the
module where implementing things yourself pays off most: small implementation details
(nearest-neighbour structures, edge validation, goal biasing, shortcutting) dominate
performance.

## Learning objectives

* Explain why sampling sidesteps explicit $\mathcal C_{obs}$, and what probabilistic
  completeness and asymptotic optimality mean (and require).
* Implement **PRM**, **RRT**, **RRT-Connect**, **RRT\***, **Informed RRT\*** and
  understand **BIT\*** and lazy variants.
* Handle the practical machinery: metrics, nearest neighbours, edge validation, goal
  sampling, narrow passages, shortcutting and smoothing.
* Use **OMPL** directly, and **benchmark** planners properly (Planner Arena,
  MotionBenchMaker).

## Key ideas to own

* **The idea.** Sample configurations, keep the collision-free ones, connect nearby
  samples with valid local motions, and search the resulting graph or tree. You only
  ever need a collision oracle, so high dimension costs samples, not grid cells.
* **PRM** (multi-query): build a roadmap once, then answer many queries. **Lazy PRM**
  defers edge checks until an edge lies on a candidate path, which is a big win when
  checks are expensive.
* **RRT** (single-query): extend the tree from the nearest node towards a random sample,
  by a step $\eta$. The **Voronoi bias** pulls growth into unexplored space. Add goal
  biasing (sample the goal with probability $p$).
* **RRT-Connect:** two trees (from start and goal), with a greedy "connect" that extends
  repeatedly until blocked. It is simple and very fast for manipulators, and it is what
  you'll use most. It gives no optimality.
* **Guarantees.** *Probabilistic completeness* holds for problems with $\delta$-clearance
  paths: the failure probability decays exponentially in the number of samples. RRT
  converges to a **sub-optimal** path with probability one. **RRT\*/PRM\*** (Karaman &
  Frazzoli) connect within a radius $r_n \propto (\log n / n)^{1/d}$ and **rewire**,
  which makes them **asymptotically optimal**. The radius comes from
  random-geometric-graph connectivity.
* **Informed** planners. Once a solution of cost $c$ exists, only states in the
  prolate hyperspheroid $\{x : \|x - x_s\| + \|x - x_g\| \le c\}$ can improve it
  (Euclidean cost), so sample there (Informed RRT\*). **BIT\*** processes batches of
  samples as an implicit random geometric graph, searched in order of heuristic
  promise. AIT\* and EIT\* refine the heuristics further.
* **Narrow passages** are the enemy. Remedies: Gaussian and bridge-test sampling,
  obstacle-based sampling, retraction, and bidirectional search.
* **Nearest neighbours.** $k$-d trees break with wrap-around joints and non-Euclidean
  metrics, which is why OMPL uses GNAT. A SIMD linear scan is often fastest for
  $< 10^4$ nodes (VAMP).
* **Post-processing.** Random shortcutting, vertex reduction and B-spline smoothing
  (re-validated), then **time parameterisation** (Module 13). Planner output is a
  geometric path, not a trajectory.
* **Benchmarking.** Same scenes, same collision checker, many seeds, a time limit, and
  the full distributions of time to first solution, success rate, cost and collision
  checks. OMPL's benchmark format plus Planner Arena does this for you.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| LaValle, *Planning Algorithms* ([free](https://lavalle.pl/planning/)), Ch. 5 | Sampling theory, PRM, RRT, metrics, collision-checking strategies |
| Lynch & Park, *Modern Robotics*, Ch. 10.5 + videos | A compact introduction to RRT and PRM |
| S. Karaman & E. Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning", IJRR 2011 ([arXiv:1105.1186](https://arxiv.org/abs/1105.1186)) | RRT\*, PRM\*, the optimality proofs. Read Sections 1–4 carefully |
| [OMPL](https://ompl.kavrakilab.org/) tutorials and I. Şucan, M. Moll & L. Kavraki, "The Open Motion Planning Library", IEEE RAM 2012 | Architecture: state spaces, validity checkers, planners, benchmarking |
| A. Orthey, C. Chamzas & L. Kavraki, "Sampling-Based Motion Planning: A Comparative Review", *Annu. Rev. Control Robot. Auton. Syst.* 2024 | An up-to-date map of the field |

**Deeper**

| Resource | Take from it |
|---|---|
| L. Kavraki, P. Švestka, J.-C. Latombe & M. Overmars, "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces", IEEE TRA 1996 | PRM from the source |
| J. Kuffner & S. LaValle, "RRT-Connect: An Efficient Approach to Single-Query Path Planning", ICRA 2000 | The workhorse |
| J. Gammell, S. Srinivasa & T. Barfoot, "Informed RRT\*" (IROS 2014) and "Batch Informed Trees (BIT\*)" (ICRA 2015; IJRR 2020) | Informed sampling and batch search |
| M. Moll, I. Şucan & L. Kavraki, "Benchmarking Motion Planning Algorithms", IEEE RAM 2015 + [Planner Arena](https://plannerarena.org/) | How to benchmark |
| C. Chamzas et al., "MotionBenchMaker: A Tool to Generate and Benchmark Motion Planning Datasets", RA-L 2022 | Standard Panda/Fetch scenes and datasets |
| W. Thomason, Z. Kingston & L. Kavraki, VAMP (ICRA 2024) + [code](https://github.com/kavrakilab/vamp) | Microsecond planning through vectorisation |

## Warm-ups

1. **RRT in 2-D.** Implement RRT for a point robot among polygons. Visualise the Voronoi
   bias, then vary $\eta$ and the goal bias and plot success time.
2. **RRT vs RRT-Connect** in a 2-D maze with a narrow passage: plot time-to-solution
   CDFs over 1,000 seeds.
3. **PRM on your 2R C-space** (Module 14). Answer 100 queries on one roadmap and compare
   the total time with 100 single-query RRT-Connect runs.
4. **Optimality, visibly.** Run RRT\* on the 2R problem, plot the best cost against time
   on log axes, and draw the A\* optimum from Module 14 as a horizontal line. Then add
   informed sampling and compare.
5. **OMPL hands-on.** Use OMPL's Python bindings to plan for a point in 2-D and for a
   rigid body in SE(3). Run OMPL's benchmark tool and open the results in Planner Arena.

## Mini project: `mamp_planners`, 7-DoF planners in C++ benchmarked against OMPL *(A: RRT-Connect + benchmark; B: everything)*

**Planners:** RRT, RRT-Connect, PRM (with a lazy variant), RRT\*, Informed RRT\*
(stretch: BIT\*).

**Infrastructure:** joint-limited state space with a weighted metric; nearest neighbours
(a $k$-d tree **and** a SIMD linear scan, benchmarked against each other); collision via
`mamp_collision` (Module 15) or, for Track A, FCL through a thin wrapper; certified or
discretised edge validation (a switch); shortcutting and B-spline smoothing with
re-validation; time parameterisation with TOPP-RA (Module 13); execution via
`FollowJointTrajectory` on `mamp_bringup`.

**Problems.** At least four FR3 scenes: a table pick, a **shelf** (narrow), a box/cage,
and a cluttered tabletop. Build them yourself or adapt MotionBenchMaker's Panda scenes
(the Panda is the `fer` model in `franka_description`, so you can test both arms).

**Fair comparison.** Wrap the *same* collision checker as an OMPL
`StateValidityChecker`, run OMPL's planners and yours through the same harness, and
write OMPL's benchmark log format so both appear in Planner Arena.

**Proof of competence**

* Your RRT-Connect: ≥ 99% success within 1 s on every scene, with median time within
  1.5× of OMPL's `RRTConnect` using the same checker. If you're slower, the profile
  explains why.
* RRT\* and Informed RRT\* cost-against-time curves alongside OMPL's equivalents.
* Planner Arena plots, and a two-page analysis of which planner wins where, and why
  (narrow passages, clutter, metric choice, nearest-neighbour cost).
* Executed, time-parameterised plans visualised in RViz for every scene.

**Stretch**

* BIT\*. Compare its anytime behaviour with Informed RRT\*.
* VAMP-style vectorisation: batch FK + sphere collision across SIMD lanes for edge
  validation. Aim for sub-millisecond median RRT-Connect on the shelf scene.
* Multi-threaded RRT-Connect (parallel trees), with the speed-up reported against
  thread count.

## Self-check

1. What is the Voronoi bias, and why does it make RRT explore efficiently?
2. Why is RRT-Connect so effective for manipulators, and on what kind of problem does it
   struggle?
3. State the assumption behind probabilistic completeness and sketch why RRT satisfies it.
4. Why is RRT not asymptotically optimal? What does RRT\* change, and why does the radius
   scale like $(\log n/n)^{1/d}$?
5. Describe Informed RRT\*'s informed set. Why is it an ellipsoid?
6. When does lazy collision checking pay off?
7. What must be identical across planners for a benchmark to be fair?

Next: [Module 17: Optimisation-based planning](17-optimisation-based-planning.md)
