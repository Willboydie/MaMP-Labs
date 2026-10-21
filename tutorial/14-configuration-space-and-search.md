# Module 14: Configuration space and graph-search planning

> **Part VI · Motion planning** · Est. 15–20 h · Prerequisites: 08 · Track A

## Why this matters / where it lives in this repo

Lab 2 step 1 notes that its IK-based pipeline has "no regard for collisions", and step 4
asks how you'd cut shapes "with obstacles in the robot's workspace". Motion planning
starts here: obstacles live in the **workspace**, but the robot moves in
**configuration space** (C-space), and that is where planners search. For the FR3,
C-space is a 7-D box (the joint limits) that you can never draw. This module builds the
intuition in 2–3-D, where you can see everything, and then confronts the curse of
dimensionality that motivates sampling-based planning (Module 16).

## Learning objectives

* Define C-space, $\mathcal C_{obs}$ and $\mathcal C_{free}$ and their topology (ℝⁿ,
  tori $T^n$, SE(2), SE(3)), and choose sensible metrics on them.
* Map workspace obstacles into C-space (Minkowski sums for translating robots; sampling
  for arms).
* Explain completeness notions and the computational complexity of planning.
* Implement and analyse **graph search**: Dijkstra, A*, weighted A*, ARA*, D* Lite, plus
  lattice and motion-primitive search.
* Implement **potential-field** planners, explain their local minima, and know what
  navigation functions fix.

## Key ideas to own

* **C-space.** One point per robot configuration. Revolute joints without limits are
  circles, so a 2R arm without limits lives on a torus and your grid and metric must wrap.
  With limits (the FR3), C-space is a box in ℝ⁷. A free-flying body lives in SE(3).
* **$\mathcal C_{obs}$** is the set of configurations where the robot intersects an
  obstacle or itself. For a translating convex polygon robot $A$ among convex obstacles
  $O$, it is the Minkowski sum $O \oplus (-A)$. For arms it is curved and non-convex, and
  in 7-D it can't be computed explicitly. Planners query a **collision checker** as an
  oracle (Module 15).
* **Complexity.** The generalised piano-mover's problem is PSPACE-hard (Reif). Complete
  algorithms (Canny's roadmap, cylindrical decomposition) exist but are impractical,
  hence the weaker guarantees:
  * *complete:* finds a path if one exists, otherwise reports failure;
  * *resolution complete:* complete at the chosen discretisation;
  * *probabilistically complete:* the probability of finding an existing path tends to 1
    as running time grows (sampling, Module 16).
* **Combinatorial planning** (exact, low-dimensional): visibility graphs (shortest paths
  among polygons), trapezoidal cell decomposition, generalised Voronoi roadmaps
  (maximum clearance).
* **Graph search.**
  * A* with an admissible heuristic returns optimal paths. A *consistent* heuristic
    means no node is re-expanded.
  * Weighted A* ($f = g + \varepsilon h$) is $\varepsilon$-suboptimal and often far
    faster.
  * ARA* is anytime: it quickly finds an $\varepsilon$-bounded solution and then
    tightens the bound.
  * D* Lite repairs the solution incrementally when edge costs change (moving obstacles,
    new sensor data).
  * For manipulators, search over **motion primitives** (small joint moves) with good
    heuristics (e.g. end-effector distance via a 3-D BFS) is competitive in structured
    scenes (Cohen, Chitta & Likhachev).
* **Curse of dimensionality.** At 1° resolution, the FR3's C-space grid has about
  $360^7 \approx 8\times 10^{17}$ cells. Grids die beyond 3–4 dimensions, while sampling
  doesn't need to enumerate.
* **Potential fields (Khatib).** Attractive to the goal plus repulsive from obstacles,
  following $-\nabla U$. Fast and reactive, but **local minima** trap them.
  *Navigation functions* (Rimon & Koditschek) have a unique minimum on "sphere worlds".
  Potential fields survive today as reactive layers and as cost terms in trajectory
  optimisation (Module 17).

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| S. M. LaValle, *Planning Algorithms* ([free online](https://lavalle.pl/planning/)), Ch. 2 (discrete planning), 4 (C-space), 6 (combinatorial planning) | The definitive free text for this whole Part |
| Lynch & Park, *Modern Robotics*, Ch. 10 + videos | A compact overview of C-space, grids, sampling, potential fields and optimisation |
| H. Choset et al., *Principles of Robot Motion* (MIT Press, 2005), Ch. 2–7 | Bug algorithms, potential functions, roadmaps, cell decompositions, worked examples |
| [Red Blob Games: Introduction to A*](https://www.redblobgames.com/pathfinding/a-star/introduction.html) | Interactive intuition for BFS, Dijkstra and A* |

**Deeper**

| Resource | Take from it |
|---|---|
| J.-C. Latombe, *Robot Motion Planning* (1991) | The classic exact-planning text |
| S. Koenig & M. Likhachev, "D* Lite", AAAI 2002; M. Likhachev, G. Gordon & S. Thrun, "ARA*: Anytime A* with Provable Bounds on Sub-Optimality", NIPS 2003 | Incremental and anytime search |
| B. Cohen, S. Chitta & M. Likhachev, "Search-based Planning for Manipulation with Motion Primitives", ICRA 2010 | Search-based planning for arms |
| O. Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots", IJRR 1986; E. Rimon & D. Koditschek, "Exact Robot Navigation Using Artificial Potential Functions", IEEE TRA 1992 | Potential fields and navigation functions |
| CMU 16-782, *Planning and Decision-making in Robotics* (M. Likhachev): lecture slides | Search-based planning taught by its main proponent |

## Warm-ups

1. **See C-space.** For a planar 2R arm (link lengths 1 and 0.8) with three polygonal
   obstacles, sample $\mathcal C_{obs}$ on a 360×360 grid and render it as an image on the
   torus. Move an obstacle and watch the C-space shape change.
2. **Minkowski sums.** Compute $\mathcal C_{obs}$ for a translating triangle among convex
   polygons analytically, and confirm it against brute-force sampling.
3. **Search on a torus.** Implement Dijkstra and A* on the 2R grid **with wrap-around**.
   Visualise the expanded nodes and show that A* expands a subset of Dijkstra's.
4. **The trap.** Build a potential-field planner for the 2R arm and construct a scene where
   it gets stuck. Then build a navigation function for a disc world and show it doesn't.
5. **Count.** Estimate the memory and time needed for A* on the FR3 at 5° and 1°
   resolution. Write a paragraph on what that implies.

## Mini project: C-space lab, a visualiser and a search-planner zoo *(A: parts 1–2; B: part 3)*

**Part 1: interactive C-space visualiser.** A side-by-side app (matplotlib, pygame or a
small web app) showing the workspace of a planar 2R/3R arm with draggable polygonal
obstacles, and its C-space (a torus image for 2R; slices or voxels for 3R). Show the path
in both views and animate the arm along it.

**Part 2: planner zoo** on the discretised C-space: Dijkstra, A*, weighted A*, ARA*
(with anytime improvement visualised), D* Lite (drag an obstacle mid-execution and
replan), a potential field and a navigation function. All handle torus topology and
joint limits.

**Part 3: search in 7-D.** For the FR3 in a simple scene (a table and a box), plan with
weighted A* over joint motion primitives, using your FK and a sphere-based collision
approximation (a preview of Module 15). Use an end-effector heuristic from a 3-D BFS
over the workspace. Report expansions and time against $\varepsilon$, and find a scene
(a narrow passage) where it collapses.

**Proof of competence**

* Tests: A* cost equals Dijkstra cost on 1,000 random problems. Weighted A* cost is at
  most $\varepsilon\times$ optimal. ARA*'s reported bound always holds. D* Lite after a
  change returns the same cost as A* from scratch, with fewer expansions (report the
  ratio).
* A screen recording of D* Lite replanning live, and of the potential field trapped.
* A short report on the 7-D experiment: where search works, where it breaks, and why.

**Stretch**

* Exact $\mathcal C_{obs}$ boundaries for the 2R arm against polygons, from analytic
  contact conditions (edge–vertex and vertex–edge), overlaid on the sampled image.
* A state-lattice planner for a car-like robot (a preview of Module 18).

## Self-check

1. Define $\mathcal C$, $\mathcal C_{obs}$ and $\mathcal C_{free}$ for the FR3. What is its
   topology with and without joint limits?
2. Why can't $\mathcal C_{obs}$ be computed explicitly for a 7-DoF arm, and what do
   planners use instead?
3. Admissible vs consistent heuristics: why does consistency matter?
4. What does weighted A* guarantee? What does ARA* add?
5. When does D* Lite beat replanning from scratch?
6. Why do potential fields have local minima, and how do navigation functions avoid them?
7. Define complete, resolution complete and probabilistically complete.

Next: [Module 15: Collision detection](15-collision-detection.md)
