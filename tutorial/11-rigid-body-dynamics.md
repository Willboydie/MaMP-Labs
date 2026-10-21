# Module 11: Rigid-body dynamics

> **Part IV · Dynamics and control** · Est. 25–35 h · Prerequisites: 09 · Track B

## Why this matters / where it lives in this repo

Kinematics says where the robot *can* be. Dynamics says what it *takes* to get there:
torques, energy, and what happens when you push or let go. The data is already in this
repo:

* [`robots/fr3/inertials.yaml`](../franka_description/robots/fr3/inertials.yaml): mass,
  COM and inertia tensor per link.
* [`robots/fr3/dynamics.yaml`](../franka_description/robots/fr3/dynamics.yaml): per-joint
  friction and damping-like parameters, rendered into `<dynamics ...>` tags by the
  `franka-dynamics` macro in
  [`utils.xacro`](../franka_description/robots/common/utils.xacro). Only some of those
  attributes are standard URDF; working out which, and who reads the rest, is a warm-up.
* `effort` limits in `joint_limits.yaml`: 87 N·m for joints 1–4 and 12 N·m for joints
  5–7.

Lab 2 uses mock hardware that ignores dynamics entirely: commanded positions *become*
the state. Modules 12 and 20 replace that with physics, and this module gives you the
equations behind it.

## Learning objectives

* Derive the **manipulator equation** by Lagrange and by Newton–Euler, and state its
  structural properties.
* Use **spatial vector algebra** (Featherstone) and implement **RNEA**, **CRBA** and
  **ABA**.
* Exploit **linearity in the inertial parameters** for identification, and enforce
  **physical consistency**.
* Integrate the dynamics numerically and reason about energy behaviour and stiffness.
* Know Pinocchio well enough to use it as ground truth and as a production library.

## Key ideas to own

* **Manipulator equation:**
  $M(q)\ddot q + C(q,\dot q)\dot q + g(q) + \tau_f(\dot q) = \tau + J(q)^\top F_{ext}$.
  * $M$ is symmetric positive-definite and bounded.
  * With Christoffel-symbol $C$, the matrix $\dot M - 2C$ is skew-symmetric. This
    *passivity* property underpins most stability proofs in Module 12.
  * The equation is linear in the inertial parameters:
    $\tau = Y(q,\dot q,\ddot q)\,\pi$, with 10 parameters per link
    $(m,\ m c,\ I_{xx},\dots,I_{zz})$.
* **Spatial vectors.** 6-D motion vectors (twists) and force vectors (wrenches); spatial
  inertia $\mathcal I = \begin{bmatrix}\bar I_c + m[c]_\times[c]_\times^\top & m[c]_\times\\ m[c]_\times^\top & m\mathbf 1\end{bmatrix}$;
  the cross products $\times$ and $\times^*$; Plücker transforms. They turn the
  recursive algorithms into short loops. Featherstone orders vectors (angular, linear),
  as in this inertia formula; Pinocchio and Solà use (linear, angular). Check the
  ordering every time you compare against a library.
* **RNEA** (inverse dynamics, $O(n)$). A forward pass propagates velocities and
  accelerations from the base, with the base acceleration set to $-g$ to include gravity
  for free. A backward pass accumulates forces to the base and projects them onto the
  joint axes. Special cases: $g(q) = \mathrm{RNEA}(q,0,0)$, and
  $C\dot q + g = \mathrm{RNEA}(q,\dot q,0)$.
* **CRBA** ($O(n^2)$) builds $M(q)$ from composite rigid-body inertias. **ABA**
  ($O(n)$) computes forward dynamics $\ddot q = \mathrm{FD}(q,\dot q,\tau)$ without
  forming $M$. For a 7-DoF arm, Cholesky on CRBA's $M$ is competitive; for long chains
  ABA wins.
* **Identification.** Not all $10n$ parameters affect the torques. Only **base
  parameters** (linear combinations) are identifiable, found by QR or SVD of a stacked
  regressor. Excitation trajectories (finite Fourier series) are optimised for the
  condition number of the regressor. Plain least squares can return physically impossible
  inertias, so constrain the problem with LMIs on the pseudo-inertia (Wensing et al.).
  Gaz et al. did exactly this for the Franka Panda.
* **Friction.** Viscous, Coulomb and Stribeck terms. It dominates at low speed and in the
  wrist, and it's where most model–reality mismatch hides.
* **Integration.** Explicit Euler injects energy into oscillators. Semi-implicit
  (symplectic) Euler conserves a shadow energy, and RK4 is accurate but not symplectic.
  Stiff contacts and high-gain controllers force small steps or implicit methods
  (Module 20).

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 8 + videos ([free](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)) | Lagrangian and Newton–Euler dynamics in twist/wrench language; task-space dynamics |
| R. Featherstone, *Rigid Body Dynamics Algorithms* (Springer, 2008), Ch. 1–7, and his [spatial vector notes and slides](http://royfeatherstone.org/spatial/) | Spatial algebra, RNEA, CRBA, ABA: the definitive text |
| Siciliano et al., Ch. 7 | Lagrange formulation, properties, Newton–Euler, dynamic parameter identification |
| Tedrake, *Underactuated Robotics*, the manipulator-equations material ([underactuated.mit.edu](https://underactuated.mit.edu/)) | A compact, practical view with Drake examples |

**Deeper**

| Resource | Take from it |
|---|---|
| Carpentier et al., "The Pinocchio C++ library: a fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives", IEEE/SICE SII 2019 + [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Production-grade algorithms and analytical derivatives (used in Module 17) |
| C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano & A. De Luca, "Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization", IEEE RA-L 2019 | Identification of *this* robot family, including friction |
| Wensing, Kim & Slotine, IEEE RA-L 2018 (Module 07) | LMI constraints for physical consistency |
| W. Khalil & E. Dombre, *Modeling, Identification and Control of Robots* (2002) | Base parameters, excitation design, identification practice |
| Hairer, Lubich & Wanner, *Geometric Numerical Integration* (Springer) | Why symplectic integrators behave the way they do |

## Warm-ups

1. **2R by hand.** Derive $M$, $C$ and $g$ for a planar 2R arm with SymPy via Lagrange.
   Verify numerically that $\dot M - 2C$ is skew-symmetric. Show that a non-Christoffel
   $C$ (with the same $C\dot q$) breaks the property.
2. **Gravity at home.** Compute the FR3's $g(q)$ at the ready pose with Pinocchio. Which
   joint is closest to its effort limit, and with what payload would it saturate?
3. **Regressor.** Build $Y(q,\dot q,\ddot q)$ for the 2R arm and verify
   $\tau = Y\pi$ on random states. Find its base parameters.
4. **`<dynamics>` audit.** Which of `D`, `K`, `damping`, `friction`, `mu_coulomb` and
   `mu_viscous` are in the URDF specification? What does `urdfdom` do with the rest, and
   which consumers might read them?
5. **Energy drift.** Simulate a frictionless 2R arm released from rest with explicit
   Euler, semi-implicit Euler and RK4 at $\Delta t =$ 1 ms and 10 ms. Plot total energy
   over 60 s and explain each curve.
6. **Spatial inertia.** Build the 6×6 spatial inertia of `fr3_link1` about the *link
   frame* from `inertials.yaml` (mind the COM offset). Check that it is positive-definite.

## Mini project: `mamp_dynamics`, Featherstone from scratch, validated against Pinocchio *(B)*

Write a C++ library (on `mamp_liegroups`, models loaded from URDF) providing:
RNEA; $g(q)$; $C(q,\dot q)\dot q$; CRBA; ABA; $M^{-1}$ (via ABA or Cholesky); kinetic
and potential energy; centre of mass and its Jacobian; and the regressor
$Y(q,\dot q,\ddot q)$.

Build a small **simulator** on top: fixed-step integration (semi-implicit Euler and
RK4), joint viscous and Coulomb friction (smoothed), and joint-limit springs. Bind it to
Python.

**Proof of competence**

* Agreement with Pinocchio to **≤ 1e-10** (relative) for RNEA, CRBA and ABA on 10,000
  random states of the FR3. Identity tests: $\mathrm{FD}(q,\dot q,\mathrm{ID}(q,\dot q,\ddot q)) = \ddot q$;
  $M = M^\top \succ 0$; $\dot M - 2C$ skew-symmetric (with $\dot M$ by finite
  differences); and the power balance $\frac{d}{dt}E = \dot q^\top(\tau - \tau_f)$.
* The FR3 released from the ready pose without friction conserves energy to a stated
  tolerance over 60 s with the symplectic integrator at 1 kHz. Show the RK4 and
  explicit-Euler drift for comparison.
* A timing table (ns/call) against Pinocchio. Being within 2–5× of Pinocchio is a
  respectable first implementation.

**Stretch: identification study.**
* Design Fourier-series excitation trajectories within the FR3's limits that minimise
  the condition number of the stacked base regressor.
* Simulate noisy torques from "true" parameters and identify them with (a) ordinary
  least squares and (b) a semidefinite program (cvxpy) with pseudo-inertia LMIs.
* Show that (a) can produce inconsistent inertias while (b) cannot, then use (b) to
  repair the inconsistent link you found in Module 07.

## Self-check

1. Write the manipulator equation and three structural properties used in control proofs.
2. Why is RNEA $O(n)$ but CRBA $O(n^2)$? What trick makes ABA $O(n)$?
3. How do you get $g(q)$ and $C\dot q$ out of an RNEA routine?
4. What does "linear in the inertial parameters" mean, and why are only base parameters
   identifiable?
5. Why does explicit Euler gain energy on a pendulum, and what does semi-implicit Euler do
   differently?
6. Where does unmodelled friction show up when you compare simulation with a real Franka?

Next: [Module 12: Control and ros2_control](12-control-and-ros2-control.md)
