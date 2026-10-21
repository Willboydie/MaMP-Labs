# Module 15: Collision detection and distance computation

> **Part VI · Motion planning** · Est. 20–25 h · Prerequisites: 14 · Track B

## Why this matters / where it lives in this repo

Sampling-based planners spend most of their time asking one question: *is this
configuration (or this motion) collision-free?* Trajectory optimisers ask a harder one:
*how far is it from collision, and in which direction?* The FR3 description in this repo
ships three geometric models of the same robot, each built for a different consumer:

* `meshes/robot_arms/fr3/visual/*.dae`: detailed, for rendering only;
* `meshes/robot_arms/fr3/collision/*.stl`: simplified meshes, for environment collision;
* self-collision **capsules** (cylinder + two spheres), inflated by a 3 cm
  `safety_distance`, in
  [`franka_arm.xacro`](../franka_description/robots/common/franka_arm.xacro) and
  [`utils.xacro`](../franka_description/robots/common/utils.xacro) (enabled with
  `with_sc:=true`). These mirror the coarse model Franka's own controller uses.

## Learning objectives

* Explain the **broad-phase / narrow-phase** pipeline and the main bounding-volume
  hierarchies.
* Implement exact distance and penetration for primitives, and **GJK/EPA** for convex
  shapes.
* Build robot collision models (capsules, spheres, convex decompositions) and an
  **allowed collision matrix**.
* Validate *motions*, not just states: discretised vs conservative or continuous checking.
* Compute **signed distance and its gradient** with respect to joint angles, for
  optimisation.
* Use FCL/Coal fluently, and know what makes modern checkers such as VAMP fast.

## Key ideas to own

* **Pipeline.** The *broad phase* (AABB trees, sweep-and-prune, spatial hashing) culls
  pairs that can't touch. The *narrow phase* computes exact contact or distance for the
  survivors. A robot-against-scene check is many link–object pairs, and self-collision
  adds link–link pairs minus those in the allowed collision matrix (adjacent links, and
  pairs that never collide).
* **Primitives first.** Sphere–sphere is trivial. Capsule–capsule reduces to
  segment–segment closest points; get the degenerate parallel case right (Ericson,
  Ch. 5). Capsules are *swept spheres*: cheap, smooth, and conservative.
* **GJK.** Two convex sets intersect iff the origin lies in their Minkowski difference
  $A \ominus B$. GJK walks a simplex towards the origin using only **support functions**
  $s_A(d) = \arg\max_{x\in A} d^\top x$, so it works for any convex shape with a support
  function. It returns the distance and witness points when the sets are separated.
  **EPA** expands the final simplex to get the penetration depth when they overlap.
  Recent work accelerates GJK with Nesterov momentum (Montaut et al.; in Coal).
* **Meshes.** Mesh–mesh checks use BVHs over triangles (OBB/RSS trees). Non-convex
  meshes are often replaced by convex decompositions (V-HACD, CoACD) so GJK applies.
* **Sphere models.** Approximating each link by 10–50 spheres makes checks
  embarrassingly parallel and differentiable. cuRobo (GPU) and VAMP (CPU SIMD) both rely
  on this, and VAMP adds a point-cloud structure (CAPT) with nanosecond queries.
* **Motion validation.** Planners check edges by sampling at a resolution (OMPL's
  "longest valid segment fraction"), which can **tunnel** through thin obstacles. A
  *conservative* check uses a bound on how far any robot point can move per unit joint
  motion (a Lipschitz bound built from link radii and lengths), stepping by
  $\text{clearance}/\text{bound}$: *conservative advancement*. Fully continuous collision
  detection (CCD) computes the time of first contact.
* **Signed distance for optimisation.** Optimisers need $d(q)$ and $\nabla_q d$: with
  witness points $p_A$, $p_B$ and normal $n$,
  $\nabla_q d = n^\top\big(J_{p_A}(q) - J_{p_B}(q)\big)$, using positional Jacobians
  (Module 09). Precomputed signed distance fields (voxel grids) give fast environment
  distances; CHOMP uses them.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| C. Ericson, *Real-Time Collision Detection* (Morgan Kaufmann, 2004), Ch. 4–6 and 9 | Bounding volumes, primitive tests, BVHs, GJK. The practitioner's bible |
| E. G. Gilbert, D. W. Johnson & S. S. Keerthi, "A Fast Procedure for Computing the Distance Between Complex Objects in Three-Dimensional Space", IEEE J. Robotics & Automation, 1988 | The original GJK paper |
| Casey Muratori, "Implementing GJK" (2006 lecture video; search YouTube for the title) | The best intuitive explanation of GJK |
| J. Pan, S. Chitta & D. Manocha, "FCL: A General Purpose Library for Collision and Proximity Queries", ICRA 2012 + [FCL](https://github.com/flexible-collision-library/fcl) | The library MoveIt uses |

**Deeper**

| Resource | Take from it |
|---|---|
| [Coal](https://github.com/coal-library/coal) (formerly HPP-FCL) and Montaut et al., "Collision Detection Accelerated: An Optimization Perspective", RSS 2022 | A modern, faster GJK/EPA. Used by Pinocchio |
| G. van den Bergen, *Collision Detection in Interactive 3D Environments* (2003) | A GJK implementation in depth |
| W. Thomason, Z. Kingston & L. Kavraki, "Motions in Microseconds via Vectorized Sampling-Based Planning", ICRA 2024 + [VAMP](https://github.com/kavrakilab/vamp) | SIMD collision checking and CAPT for point clouds |
| B. Sundaralingam et al., "cuRobo: Parallelized Collision-Free Robot Motion Generation", ICRA 2023 + [curobo.org](https://curobo.org/) | GPU sphere models and signed distance |
| [CoACD](https://github.com/SarahWeiii/CoACD) (Wei et al., SIGGRAPH 2022) | Collision-aware convex decomposition |
| A. Hornung et al., "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees", Autonomous Robots 2013 | Collision against sensed environments |

## Warm-ups

1. **Segment–segment.** Implement robust closest points between segments (handle
   parallel and degenerate cases), then capsule–capsule distance. Test 10⁶ random pairs
   against brute-force sampling.
2. **See the capsules.** Generate the FR3 URDF with `with_sc:=true` and display the
   capsules in RViz. List all link pairs, then find pairs that never collide within the
   joint limits by sampling 10⁶ configurations (this is what the MoveIt Setup Assistant
   does).
3. **2-D GJK, step by step.** Implement GJK for convex polygons and animate the simplex
   evolving towards the origin.
4. **Cost of fidelity.** With Coal or FCL, time collision checks of the FR3 against a
   box-cluttered scene using (a) collision meshes, (b) convex hulls and (c) capsules.
   Report the speed ratios and where the models disagree.
5. **Tunnelling.** Build a scene with a 5 mm-thick obstacle and show that discretised edge
   checking at a typical resolution misses a collision. Then compute the conservative
   step size from a Lipschitz bound and show that it catches it.

## Mini project: `mamp_collision`, fast collision and distance for the FR3 *(B)*

Write a ROS-free C++ library (with Python bindings) that you'll plug into your planners
in Modules 16–18.

**Requirements**

1. Primitives (sphere, capsule, box, convex hull), GJK + EPA with witness points, and an
   AABB tree for the environment.
2. Two robot models: (a) Franka's capsules; (b) a sphere model you fit automatically to
   the collision meshes (e.g. a greedy covering with a bounded radius error). Report how
   many spheres per link and the error.
3. Self-collision with an automatically generated allowed collision matrix.
4. **Batched** queries: `check(qs[N×7]) -> bool[N]`. Vectorise with Eigen and/or SIMD
   intrinsics.
5. **Certified motion validation** between $q_a$ and $q_b$ by conservative advancement.
6. **Signed distance and gradient** $\nabla_q d(q)$ for the minimum-distance pair, and
   a smooth soft-min over pairs for optimisers.

**Proof of competence**

* Against Coal/FCL: 100% boolean agreement on 10⁶ random convex pairs, and distance
  agreement to ≤ 1e-9 when separated.
* Against your own exact model: certified motion validation has **zero misses** in
  10⁵ adversarial thin-obstacle edges. Report how many checks it costs relative to
  discretised validation.
* Gradient checked by finite differences (O(h²) plot).
* Throughput of the capsule model for the FR3 against 50 boxes, single core. Aim for
  **≥ 10⁶ configuration checks per second**; report what you achieve, with a profile
  showing where time goes.

**Stretch**

* Point-cloud environments: a k-d tree, then a CAPT-like structure. Compare with VAMP.
* A JAX or CUDA batched sphere checker on the GPU: 10⁷ checks per second.
* Exact CCD for capsules under linear joint interpolation.

## Self-check

1. Why is collision checking the bottleneck of sampling-based planners?
2. Explain broad phase vs narrow phase, with an example structure for each.
3. What is a support function, and why does GJK operate on the Minkowski difference?
4. Why does Franka model self-collision with capsules?
5. How can discretised edge checking miss a collision, and how do you make it conservative?
6. Why do trajectory optimisers need signed distance and a gradient rather than a boolean?
   How do you compute $\nabla_q d$?
7. Compare mesh, convex-decomposition and sphere models: accuracy, speed and
   differentiability.

Next: [Module 16: Sampling-based planning](16-sampling-based-planning.md)
