# Module 09: Differential kinematics, Jacobians, singularities and statics

> **Part III · Kinematics** · Est. 15–20 h · Prerequisites: 08 · Track A

## Why this matters / where it lives in this repo

Lab 2's IK solver (`PyKDL.ChainIkSolverPos_LMA`) is a Jacobian-based iteration. Lab 2,
step 2 warns that "not all Cartesian points can be reached", and some *reachable* points
are still bad because the arm is near a **singularity**. The plasma-cutter question in
step 4 depends on how Cartesian velocity maps to joint velocity along a path. Jacobians
are also how forces map to torques (statics), how controllers work in task space
(Module 12) and how trajectory optimisers get gradients (Module 17).

## Learning objectives

* Define and compute **space, body and "geometric" (mixed-frame) Jacobians**, and
  convert between them with adjoints.
* Distinguish the **geometric** from the **analytic** Jacobian, and explain
  representation singularities.
* Analyse **singularities** with the SVD; compute manipulability, condition number and
  velocity/force ellipsoids.
* Use **statics**: $\tau = J^\top F$, and why it follows from virtual work.
* Verify any Jacobian numerically, with finite differences on the group.

## Key ideas to own

* **Twists and Jacobians.** $\mathcal V = J(q)\dot q$. In PoE form the space Jacobian is
  $J_{s,1} = S_1$ and $J_{s,i} = \mathrm{Ad}_{e^{[S_1]q_1}\cdots e^{[S_{i-1}]q_{i-1}}}S_i$.
  The body Jacobian is $J_b = \mathrm{Ad}_{T^{-1}}J_s$.
* **Which velocity?** The *spatial twist* is the velocity of the (imaginary) body point
  at the **base origin**, expressed in base axes. The textbook "geometric Jacobian"
  (Siciliano; Craig) gives the velocity of the **end-effector origin**, expressed in base
  axes. In columns: $J_{P,i} = z_{i}\times(p_e - p_{i})$ and $J_{O,i} = z_{i}$ for a
  revolute joint, where $z_i$ is joint $i$'s unit axis and $p_i$ any point on it, both
  in base coordinates. Textbooks using standard DH write these as $z_{i-1}$ and
  $p_{i-1}$. Pinocchio names these frames `WORLD`, `LOCAL` and
  `LOCAL_WORLD_ALIGNED`, and KDL's `ChainJntToJacSolver` returns the last of them. Always
  write down which one you mean.
* **Analytic Jacobian.** The derivative of a minimal pose parameterisation (e.g.
  position + RPY): $J_A = \mathrm{blkdiag}(I, E(\phi)^{-1})\,J$. It inherits the Euler
  angles' singularities even where the arm itself is fine.
* **Singularities.** Let $J = U\Sigma V^\top$. Rank loss ($\sigma_{\min}\to 0$) means some
  Cartesian direction ($U$'s last column) can't be produced, and joint speeds blow up
  like $1/\sigma_{\min}$ for a fixed Cartesian velocity. There are *boundary*
  singularities (arm fully stretched) and *internal* ones (axes aligning: shoulder, elbow,
  wrist). For a 7-DoF arm the null space has dimension $\ge 1$: the **self-motion**, the
  elbow "swivel" that moves the arm without moving the hand.
* **Measures.** Manipulability $w = \sqrt{\det JJ^\top} = \prod\sigma_i$ (Yoshikawa),
  condition number $\sigma_{\max}/\sigma_{\min}$, and the velocity ellipsoid
  $\{J\dot q : \|\dot q\|\le 1\}$ with semi-axes $\sigma_i u_i$. The force ellipsoid has
  inverted axes. Linear and angular rows have different units, so analyse them
  separately or scale by a characteristic length.
* **Statics.** Virtual work, $F^\top \delta x = \tau^\top\delta q$, gives
  $\tau = J^\top F$, with $J$ and $F$ in the *same* frame and ordering. Near a singularity
  some wrenches need no torque at all: a stretched arm holds a load along its length "for
  free".
* **Acceleration level.** $\ddot x = J\ddot q + \dot J\dot q$. You'll need $\dot J\dot q$
  for computed-torque and operational-space control (Modules 11–12).
* **Numerical verification.** For the body Jacobian, column $i$ is approximately
  $\log\!\big(T(q)^{-1}T(q + h e_i)\big)^\vee / h$. Use central differences and the group
  log, never subtraction of Euler angles.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 5 + videos ([free](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)) | Space/body Jacobians, statics, singularities, manipulability ellipsoids |
| Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch. 3 | Geometric vs analytic Jacobian, singularity decoupling, redundancy, manipulability |
| Stanford CS223A (Khatib), *Introduction to Robotics*, lectures on Jacobians ([Stanford Engineering Everywhere](https://see.stanford.edu/Course/CS223A)) | Classic lectures. Watch the Jacobian and statics ones |

**Deeper**

| Resource | Take from it |
|---|---|
| T. Yoshikawa, "Manipulability of Robotic Mechanisms", *IJRR* 4(2), 1985 | The original manipulability measure |
| Y. Nakamura, *Advanced Robotics: Redundancy and Optimization* (1991) | Null space and redundancy |
| [Pinocchio documentation](https://stack-of-tasks.github.io/pinocchio/): frames and Jacobians | `computeFrameJacobian` with each `ReferenceFrame`, plus `getFrameJacobianTimeVariation` |
| Spong et al., *Robot Modeling and Control*, Ch. 4 | Alternative derivations and examples |

## Warm-ups

1. **2R/3R by hand.** Derive the Jacobian of a planar 2R arm, find its singularities, and
   plot $w(q)$ as a heat map over the torus $[-\pi,\pi]^2$.
2. **Frame zoo.** For the FR3, compute the Jacobian with your PoE code (space and body),
   with KDL's `ChainJntToJacSolver` and with Pinocchio in each `ReferenceFrame`. Write
   down the exact adjoint or rotation that maps each one to the others, and verify it
   numerically.
3. **Finite differences.** Check your body Jacobian with the group-log formula above.
   Plot the error against $h$ on log–log axes and explain both arms of the V shape.
4. **Payload torques.** Compute the joint torques needed to hold a 2 kg payload at the
   flange (payload gravity only) in three poses. Compare them with the `effort` limits in
   [`joint_limits.yaml`](../franka_description/robots/fr3/joint_limits.yaml) (note how much
   weaker joints 5–7 are).
5. **Speed limits.** At a given configuration, find the maximum end-effector speed in
   the $+x$ direction (orientation held fixed) subject to the joint `velocity` limits.
   Formulate it as a linear program.

## Mini project: manipulability explorer and FR3 singularity atlas *(A: part 1; B: part 2)*

**Part 1: live explorer (C++ node, or Python with C++ bindings).** Subscribes to
`/joint_states` (index by name!), computes the Jacobian with your library, and publishes
`MarkerArray`s at the flange:
* the linear velocity ellipsoid (a scaled, rotated `SPHERE` from $U\Sigma$);
* the force ellipsoid;
* a colour that encodes $\sigma_{\min}$;
* text showing $w$, the condition number and the nearest-singularity direction.

Drive it with `joint_state_publisher_gui` and record a GIF of the ellipsoid collapsing as
you approach a singularity.

**Part 2: the singularity atlas.** Find the FR3's singular configurations numerically:
minimise $\sigma_{\min}(J(q))$ (or $\log w$) from ~10⁴ random starts, both within and
ignoring joint limits. Cluster the minima and, for each family, identify geometrically
which axes align (shoulder, elbow, wrist, boundary). Report which families are
reachable **inside** the FR3's joint limits, and show a representative of each in RViz.

**Proof of competence**

* Jacobian agreement with Pinocchio to ≤ 1e-12, and an O(h²) finite-difference plot.
* A unit test showing the ellipsoid marker's axes equal $U\Sigma$ at a known
  configuration (catches the classic transpose and column-order bugs).
* An atlas report with one figure per singularity family, and a plain-language
  explanation of why the FR3's joint limits (joints 4 and 6 especially) keep it away from
  some of them.

**Stretch**

* An interactive marker to drag the end-effector target, with resolved-rate motion
  $\dot q = J^{+}\dot x$ (a preview of Module 10) that visibly struggles near
  singularities. Add damping and show the difference.
* A **Cartesian speed map**: for a plane above the table with the tool pointing down,
  plot the maximum achievable speed in each direction. Use it to argue which letters in
  Lab 2 step 2 should be drawn fast and which slowly.

## Self-check

1. Define the space Jacobian, the body Jacobian and Siciliano's geometric Jacobian. How do
   you convert between each pair?
2. When is the analytic Jacobian singular while the geometric one is not?
3. What does $\sigma_{\min}$ tell you physically? What happens to joint speeds for a
   fixed Cartesian speed as $\sigma_{\min}\to0$?
4. Why is $\sqrt{\det JJ^\top}$ unit-inconsistent for a 6×7 Jacobian, and what can you do
   about it?
5. Derive $\tau = J^\top F$ from virtual work. Which frame must $F$ be expressed in?
6. What is the dimension of the FR3's Jacobian null space away from singularities, and
   what motion does it correspond to?

Next: [Module 10: Inverse kinematics](10-inverse-kinematics.md)
