# Module 22: Capstone, an autonomous FR3 work cell

> **Part IX · Capstone** · Est. 40–80 h · Prerequisites: as many of 00–21 as you've done · Track B

## Purpose

Everything so far was a component. The capstone is a **system**: perception →
world model → task and motion planning → trajectory generation → control → simulation,
engineered with the discipline of Modules 00–04 and evaluated like research. Done
well, it's a portfolio piece, a strong basis for your MSc dissertation, and a natural
topic for job interviews.

## Choose a theme

| Theme | Core challenge | Builds most on |
|---|---|---|
| **A. Automated cutting cell** | Plates with contours from a DXF file arrive at random poses on a table. Detect the plate pose, plan constrained cutting paths with transitions, execute at constant speed, recover from failures | 13, 15, 17, 18 |
| **B. Clutter clearing with people nearby** | Clear a cluttered table into bins: grasp selection, regrasping, constrained transport, plus a simulated human hand that moves through the workspace and forces reactive replanning (Hybrid Planning or MPC) | 16, 17, 19, 20 |
| **C. Dual-arm assembly** | Your Module 07 dual-arm torso: handover and a two-arm insertion with closed-chain constraints and impedance control during contact | 07, 12, 18, 20 |
| **D. Reproduce and extend a paper** | E.g. VAMP-style vectorised planning, GCS, Motion Policy Networks or Diffusion Policy on the FR3. Reproduce the headline result, then extend it in one clearly stated direction | 15–17, 21 |

## Requirements (all themes)

**System**
* A simulated RGB-D camera (Gazebo or MuJoCo) → point cloud → world model (an octomap in
  the MoveIt planning scene, or fitted primitives). No "god mode" object poses, except as
  a documented ablation.
* Planning through MoveIt 2 with **at least one of your own components** in the loop
  (planner plugin, IK plugin, trajectory optimiser or controller).
* Execution through `ros2_control` on simulated hardware, with **effort or impedance
  control** for any contact phase.
* Lifecycle-managed nodes, one parameterised launch, `use_sim_time` throughout.
* **Failure handling:** planning timeouts, IK failures, grasp slips, collisions detected
  at execution time, and a safe stop. Every failure path is logged and recoverable.

**Engineering**
* CI that builds, runs unit tests, and runs at least one **headless end-to-end
  `launch_testing` scenario** in simulation that asserts task success.
* Benchmarks driven by scripts, from raw logs to plots in one command.
* A README covering the architecture (with a diagram), how to run it, a design-decisions
  log, and known limitations.

**Evaluation and write-up**
* At least 50 randomised trials per condition, with success rate ± 95% confidence
  intervals, cycle time, planning and execution time breakdown, and failure taxonomy.
* At least one **ablation**: your component vs the stock MoveIt or ROS one.
* A 6–8 page paper-style report (IEEE conference format): introduction, related work,
  method, experiments, limitations and future work. Plus a 2–3 minute video.

## Suggested milestones

| Week | Milestone |
|---|---|
| 1 | Theme chosen; one-page proposal with success metrics; architecture diagram |
| 2–3 | Simulation scene, perception → planning scene, and the end-to-end "walking skeleton" (simplest version of every stage working) |
| 4–5 | Your own component(s) integrated; failure handling; first benchmark |
| 6 | CI end-to-end test; ablation experiments |
| 7–8 | Polish, full evaluation, write-up and video |

## Rubric

| Criterion | Competent | Excellent | Outstanding |
|---|---|---|---|
| **System integration** | The pipeline runs end to end in simulation | Robust over randomised trials, with clean recovery from common failures | Handles perturbations it wasn't designed for; reactive replanning |
| **Own technical contribution** | One component reimplemented and integrated | Component measurably better than the stock alternative on some axis | A novel extension with a clear, defended improvement |
| **Modelling and planning depth** | Correct use of the frameworks | Constraints, dynamics and limits handled explicitly and verified | Formal reasoning (guarantees, bounds), validated empirically |
| **Engineering** | Builds and runs from the README | CI with end-to-end tests; reproducible benchmarks | Others could extend it; clean interfaces and docs |
| **Evaluation** | Success rates reported | Confidence intervals, ablations, failure taxonomy | Insightful analysis that changes how you'd design the next system |
| **Communication** | Clear report and video | Paper-quality figures and writing | Workshop-submittable |

## Going further: real hardware

If your department has FR3s available, running the capstone on hardware via
[franka_ros2](https://github.com/frankarobotics/franka_ros2) is the ultimate test.
**Only do so with the course staff's permission and supervision, and follow the lab's
safety rules.** Start in impedance mode at low speeds, with the external enable device
in hand, and test every motion in simulation first. You'll need an RT kernel (Module 02)
and will meet every sim-to-real issue from Module 20 at once.

## Dissertation seeds

Open problems that grow naturally from this series:
* Certified-safe execution of learned motion policies (Modules 15 and 21).
* Fast constrained planning on manifolds with GPU or SIMD parallelism (Modules 16 and 18).
* Contact-rich trajectory optimisation with differentiable or hydroelastic contact
  (Modules 17 and 20).
* Planner-aware task allocation for dual-arm systems (Module 18).
* Identifying and compensating FR3 friction and elasticity for precise tool paths
  (Modules 11–13).

Back to the [roadmap](README.md) · [Master resource list](resources.md)
