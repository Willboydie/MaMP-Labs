# Module 05: Rigid-body transformations, rotations and Lie groups

> **Part II · Modelling maths** · Est. 20–25 h · Prerequisites: linear algebra · Track A

## Why this matters / where it lives in this repo

Every number a robot uses about *where* something is lives in some frame, and every
orientation has a representation with its own conventions and failure modes. In this repo:

* [`Lab1/transform_helpers/transform_helpers/utils.py`](../Lab1/transform_helpers/transform_helpers/utils.py)
  asks for three conversions: rotation matrix → ROS quaternion, quaternion → Rodrigues
  (rotation) vector, and quaternion → intrinsic Z-Y-X Tait–Bryan angles.
* URDF joint origins use `xyz` + `rpy`. See
  [`franka_description/robots/fr3/kinematics.yaml`](../franka_description/robots/fr3/kinematics.yaml),
  which is full of ±π/2 rolls.
* Lab 2's `get_joint_pos` builds `PyKDL.Rotation.RPY(r, p, y)` from orientation targets.
  The comment there spells out a convention you need to be able to verify for yourself.
* tf2 transmits orientation as a quaternion with fields `x, y, z, w`.

Errors here are silent. A transposed matrix, swapped quaternion order or wrong Euler
convention still produces a valid-looking rotation, just the wrong one.

## Learning objectives

* Manipulate SO(3) and SE(3) fluently: composition, inversion, frame-change notation,
  active vs passive interpretation.
* Convert between rotation matrices, Euler angles (any of the 12 sequences, intrinsic or
  extrinsic), axis-angle and rotation vectors, and unit quaternions, **robustly** near
  singularities.
* Use the **Lie group** view: hat/vee, exp/log, adjoint, twists, left/right Jacobians,
  and perturbations for optimisation.
* Interpolate and average rotations correctly (slerp, geodesics, hemisphere checks).
* State the conventions of ROS messages, URDF, KDL, Eigen, scipy and transforms3d from
  memory, or know exactly where to check.

## Key ideas to own

**Notation.** Write ${}^{A}T_{B}$ for the pose of frame $B$ expressed in frame $A$.
It maps coordinates: $p_A = {}^{A}T_{B}\,p_B$. Composition chains indices:
${}^{A}T_{C} = {}^{A}T_{B}\,{}^{B}T_{C}$. Inverse:
$\begin{bmatrix}R & p\\0 & 1\end{bmatrix}^{-1}=\begin{bmatrix}R^\top & -R^\top p\\0&1\end{bmatrix}$.
The columns of ${}^{A}R_{B}$ are $B$'s axes written in $A$. If you write subscripts
consistently, most frame bugs become visible on paper.

**Euler angles.**
* *Intrinsic* rotations are about the moving (new) axes; *extrinsic* rotations are
  about the fixed (original) axes. Intrinsic $Z\!-\!Y'\!-\!X''$ (yaw, pitch, roll) equals extrinsic
  $X\!-\!Y\!-\!Z$ with the angles in reverse order. Both give
  $R = R_z(\psi)\,R_y(\theta)\,R_x(\phi)$.
* URDF's `rpy` and KDL's `Rotation.RPY(r,p,y)` are roll about fixed X, then pitch about
  fixed Y, then yaw about fixed Z: $R = R_z(y)R_y(p)R_x(r)$.
* Extraction uses `atan2`, and becomes singular at pitch $= \pm\pi/2$ (**gimbal lock**),
  where only $\psi \pm \phi$ is determined. Handle that branch explicitly.
* Euler angles are for human input and output only. Never interpolate them, never
  average them, and never optimise over them.

**Axis–angle and Rodrigues.** $R = I + \sin\theta\,[k]_\times + (1-\cos\theta)[k]_\times^2$.
The rotation vector is $\omega = \theta k$. The log map uses $\theta = \arccos\!\big(\tfrac{\operatorname{tr}R - 1}{2}\big)$.
It is ill-conditioned near $\theta = 0$ (use Taylor series) and near $\theta = \pi$
(recover the axis from the symmetric part of $R$, not from $R - R^\top$).

**Unit quaternions.** $q = (\cos\tfrac{\theta}{2},\ \sin\tfrac{\theta}{2}\,k)$ in the
Hamilton convention. Composition $q_{AC} = q_{AB} \otimes q_{BC}$, and rotation
$v' = q \otimes (0, v) \otimes q^{*}$. $q$ and $-q$ represent the same rotation
(**double cover**), so before interpolating or averaging, flip one quaternion into the
same hemisphere ($q_1\cdot q_2 \ge 0$).
* *Matrix → quaternion:* the textbook $w = \tfrac12\sqrt{1+\operatorname{tr}R}$ loses
  precision as $\operatorname{tr}R \to -1$ ($\theta \to \pi$). Robust methods (Shepperd's)
  branch on the largest of $\{\operatorname{tr}R, R_{11}, R_{22}, R_{33}\}$.
* *Ordering traps:* `geometry_msgs/Quaternion` stores `x, y, z, w`. Eigen's constructor
  is `Quaterniond(w, x, y, z)` but `.coeffs()` returns `(x, y, z, w)`. scipy's
  `Rotation.from_quat` is scalar-last by default. `transforms3d` is scalar-first.
  JPL-convention papers multiply differently from Hamilton ones.

**Lie groups (the unifying view).** SO(3) and SE(3) are matrix Lie groups. Their Lie
algebras so(3) and se(3) are the tangent spaces at the identity, with the hat operator
$\omega \mapsto [\omega]_\times$ and $\xi = (v,\omega) \mapsto \begin{bmatrix}[\omega]_\times & v\\ 0 & 0\end{bmatrix}$.
* $\exp$ maps a twist to a rigid motion. That is a **screw motion** (Chasles' theorem),
  the basis of the product-of-exponentials FK in Module 08.
* The **adjoint** $\mathrm{Ad}_T = \begin{bmatrix}R & [p]_\times R\\ 0 & R\end{bmatrix}$
  (for twists ordered $(v,\omega)$) changes the frame of a twist:
  $T\exp(\hat\xi)T^{-1} = \exp(\widehat{\mathrm{Ad}_T\xi})$. It is how Jacobians and
  wrenches move between frames (Module 09).
* **Perturbations.** Right: $T \oplus \delta = T\exp(\hat\delta)$, a body-frame increment.
  Left: $\exp(\hat\delta)T$, a world-frame increment. The **left and right Jacobians** of
  SO(3)/SE(3) relate small changes in exponential coordinates to these perturbations.
  Optimisation on manifolds (IK in Module 10, trajectory optimisation in Module 17)
  updates with $\oplus$ instead of `+`.

**Numerical hygiene.** Products of many rotation matrices drift off SO(3). Re-project
with SVD (polar decomposition), or keep quaternions and renormalise. Compare rotations
with a proper metric: the geodesic angle $\|\log(R_1^\top R_2)\|$, or
$1-|q_1\cdot q_2|$ as a cheap proxy.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 3, plus its short lecture videos ([free book and videos](https://hades.mech.northwestern.edu/index.php/Modern_Robotics); also on Coursera) | SO(3), SE(3), exponential coordinates, twists, wrenches, adjoint. The cleanest modern treatment |
| Solà, Deray & Atchuthan, "A micro Lie theory for state estimation in robotics" ([arXiv:1812.01537](https://arxiv.org/abs/1812.01537)) | Exp/log, ⊕/⊖, adjoints and Jacobians, with a cheat sheet you'll use for years |
| [Ben Eater & 3Blue1Brown's interactive quaternions](https://eater.net/quaternions) | Visual intuition for quaternions and double cover |
| [QUT Robot Academy](https://robotacademy.net.au/) (Peter Corke), spatial-maths lessons | Short videos on rotations, Euler/RPY and homogeneous transforms |
| Craig, *Introduction to Robotics: Mechanics and Control* (4th ed.), Ch. 2 | Frame notation and Euler/fixed-angle conventions (the same book as the modified DH in Module 08) |

**Deeper**

| Resource | Take from it |
|---|---|
| Solà, "Quaternion kinematics for the error-state Kalman filter" ([arXiv:1711.02508](https://arxiv.org/abs/1711.02508)) | Hamilton vs JPL conventions; quaternion derivatives |
| Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors" (Stanford tech report, 2006) | An exhaustive table of conversions and their derivatives |
| Murray, Li & Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 2 ([free PDF](https://www.cds.caltech.edu/~murray/mlswiki/)) | Rigorous screw theory |
| Barfoot, *State Estimation for Robotics* (2nd ed., 2024), Ch. 7 ([author's page, free copy](https://asrl.utias.utoronto.ca/~tdb/)) | Matrix Lie groups, Jacobians, uncertainty on SE(3) |
| Libraries: [Sophus](https://github.com/strasdat/Sophus), [manif](https://github.com/artivis/manif), [scipy `Rotation`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html), [spatialmath-python](https://pypi.org/project/spatialmath-python/) | References to test your own code against |

## Warm-ups

1. **Prove** on paper that intrinsic Z-Y'-X'' equals extrinsic X-Y-Z with reversed
   angles. Then check numerically with scipy (`'ZYX'` is intrinsic, `'xyz'` is extrinsic).
2. **Convention zoo.** Take one rotation and write it as a ROS `Quaternion` message, an
   Eigen quaternion (constructor and `.coeffs()`), a scipy quaternion and a transforms3d
   quaternion. Keep the table in your notes.
3. **Precision cliff.** Implement the naive and the branch-on-largest matrix → quaternion
   methods. Plot the reconstruction error against rotation angle for 0–π. Explain the
   curve.
4. **Gimbal lock, visibly.** Interpolate between two orientations (a) linearly in Euler
   angles and (b) with slerp. Publish both as TF frames over time and watch them in RViz.
5. **Adjoint identity.** Verify $T\exp(\hat\xi)T^{-1} = \exp(\widehat{\mathrm{Ad}_T\xi})$
   numerically for random $T$ and $\xi$.
6. **Decode the Franka YAML.** For each joint in `kinematics.yaml`, write the origin
   rotation as a matrix and say which axis the next joint's $z$ points along in the
   parent frame.

## Mini project: `mamp_liegroups`, a tested SO(3)/SE(3) library *(A)*

Write a header-only C++17 library on Eigen, with pybind11 or nanobind bindings. Keep it
free of ROS; put ROS message conversions in a separate small header.

**API:** `SO3`, `SE3` (stretch: `SO2`, `SE2`, `Sim3`) with `exp`, `log`, `hat`, `vee`,
`compose`, `inverse`, `act`, `Adj`, left and right Jacobians (and inverses), geodesic
interpolation, uniform random sampling, distances, and conversions to and from: rotation
matrix; quaternion with an explicit convention tag (`ScalarFirst`/`ScalarLast`);
rotation vector; and all 12 Euler/Tait–Bryan sequences, intrinsic and extrinsic.

**Testing**
* Property-based tests (Python `hypothesis` through the bindings, or RapidCheck in C++)
  for every round trip, including adversarial inputs: θ ∈ {0, 1e-12, π − 1e-9, π},
  gimbal-lock pitches, and non-normalised quaternion input.
* Cross-checks against scipy and Sophus or manif.
* Jacobians checked by central finite differences, and by complex step where it applies.

**Proof of competence**

* 10⁶ random round trips (matrix ↔ quaternion ↔ rotation vector ↔ Euler, for all
  sequences) agree to **≤ 1e-12** (geodesic angle), **including θ = π exactly**.
* A log–log plot showing finite-difference Jacobian error falling as O(h²) until
  round-off. That is how you know the analytic Jacobian is right.
* A one-page "conventions card" in your notes covering ROS, URDF, KDL, Eigen, scipy and
  transforms3d.
* A benchmark (ns/op for compose, exp, log and act) against Eigen and Sophus.

**Stretch**

* Prove your uniform sampler is uniform: the histogram of rotation angles must match the
  density $(1-\cos\theta)/\pi$ on $[0,\pi]$ (Kolmogorov–Smirnov test).
* Quaternion averaging (Markley's eigenvector method) vs naive mean-and-normalise.
* Cumulative B-splines on SE(3) with analytic derivatives (see Sommer et al., "Efficient
  Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR 2020). You will
  reuse them for smooth Cartesian trajectories in Module 13.

## Self-check

1. Write the URDF `rpy="r p y"` rotation as a product of elementary rotations. Then write
   it as an intrinsic sequence.
2. Why do unit quaternions double-cover SO(3)? What goes wrong in slerp without a
   hemisphere check?
3. Where is the SO(3) log map numerically delicate, and how do you handle each case?
4. Left vs right perturbation: which one is a body-frame increment? Which one would you use
   to update an end-effector pose expressed in the base frame?
5. What is the adjoint of $T$, and why does it appear whenever a twist or wrench changes
   frame?
6. Give two reasons Euler angles are a poor state representation for optimisation.
7. Given two orientations, give two distance measures and the relationship between them.

Next: [Module 06: tf2 and coordinate frames](06-tf2-and-frames.md)
