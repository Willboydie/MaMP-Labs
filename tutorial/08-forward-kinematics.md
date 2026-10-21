# Module 08: Forward kinematics (DH, modified DH, product of exponentials)

> **Part III · Kinematics** · Est. 15–20 h · Prerequisites: 05, 07 · Track A

## Why this matters / where it lives in this repo

Forward kinematics (FK) is the map $q \mapsto {}^{base}T_{ee}(q)$. It underlies
everything after it: Jacobians, IK, collision checking and planning. In this repo it
appears three times, in three different conventions:

1. **Modified DH (Craig).** The table at the top of
   [`Lab1/.../main.py`](../Lab1/transform_helpers/transform_helpers/main.py) (`a_list`,
   `d_list`, `alpha_list`, 8 rows including the flange) is Franka's published MDH table.
   Lab 1 step 2 asks you to turn it into TF frames.
2. **URDF joint origins.**
   [`franka_description/robots/fr3/kinematics.yaml`](../franka_description/robots/fr3/kinematics.yaml)
   gives `xyz` + `rpy` per joint, and `robot_state_publisher` turns them into TF via KDL.
3. **KDL chains.** Lab 2 builds a `PyKDL.Chain` from the URDF between `base` and
   `fr3_link8`.

Lab 1 is graded on whether your MDH frames coincide with `robot_state_publisher`'s
URDF frames. That they *can* coincide is a theorem you should be able to prove.

## Learning objectives

* Assign frames and extract parameters in **both** DH conventions, and say exactly how
  they differ.
* Derive FK for serial chains as products of homogeneous transforms, and for trees
  recursively.
* Use the **product of exponentials** (PoE) formula in space and body forms, and
  compute screw axes directly from a URDF without assigning DH frames.
* Translate between MDH, PoE and URDF-origin representations.
* Characterise a workspace numerically and know the FR3's key dimensions and limits.

## Key ideas to own

* **Modified DH (Craig), as used by Franka:**
  ${}^{i-1}T_{i} = R_x(\alpha_{i-1})\,D_x(a_{i-1})\,R_z(\theta_i)\,D_z(d_i)$.
  Frame $i$ sits **on joint axis $i$**. $a_{i-1}$ and $\alpha_{i-1}$ describe the common
  normal between axes $i-1$ and $i$ (distance along, and twist about, $x_{i-1}$). $d_i$ and
  $\theta_i$ describe the offset and angle along and about $z_i$.
* **Standard DH (Spong/Siciliano):** ${}^{i-1}T_{i} = R_z(\theta_i)\,D_z(d_i)\,D_x(a_i)\,R_x(\alpha_i)$.
  Here frame $i$ sits on axis $i+1$. Same idea, shifted indices and a different
  operation order. Mixing the two conventions is the most common Lab 1 bug, so always
  check which convention a table uses.
* **DH's limitations.** Four parameters per joint need a careful frame choice. They are
  ill-conditioned for nearly parallel consecutive axes, which is why calibration uses
  modifications such as Hayati's. They can't place an arbitrary tool frame without an
  extra fixed transform (Franka's 8th "flange" row). URDF uses six numbers per joint and
  needs no frame-placement rules.
* **Product of exponentials.** Choose the zero configuration and the home pose $M$ of the
  end effector. Each joint $i$ has a **screw axis**: in the base frame $S_i$, and in the
  end-effector frame $B_i = \mathrm{Ad}_{M^{-1}} S_i$. For a revolute axis with unit
  direction $\omega$ through point $p$, $S = (\omega, -\omega\times p)$ in Lynch & Park's
  $(\omega, v)$ ordering. Then
  $$T(q) = e^{[S_1]q_1}\cdots e^{[S_n]q_n}\,M = M\,e^{[B_1]q_1}\cdots e^{[B_n]q_n}.$$
  PoE needs no link frames at all. Its Jacobian columns fall out directly (Module 09),
  and it is the natural language for calibration and Lie-group optimisation.
* **Tree FK** is a depth-first traversal composing parent→child transforms; that is what
  `robot_state_publisher` does. For planning you need **batched** FK (thousands of
  configurations at once). Vectorise across configurations (NumPy/JAX, or SIMD as in
  VAMP, Module 16).
* **Workspace.** The *reachable* workspace is everywhere the tool point can get to; the
  *dexterous* workspace is where it can get to with any orientation. Estimate both by
  Monte Carlo sampling of FK within joint limits. Note that the FR3's joint 4 range is
  roughly $[-3.04, -0.15]$ rad and joint 6 is $[0.54, 4.52]$, so **$q = 0$ is outside the
  joint limits** (check
  [`joint_limits.yaml`](../franka_description/robots/fr3/joint_limits.yaml)). Lab 2's
  mock hardware nonetheless starts at all-zero (`initial_value` 0.0 in `urdfs/fr3.urdf`).

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| Lynch & Park, *Modern Robotics*, Ch. 4 (FK, PoE) and Appendix C (DH), with the videos ([free](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)) | PoE, the link to DH, worked examples |
| Craig, *Introduction to Robotics* (4th ed.), Ch. 3 | **The** reference for modified DH, the convention Franka uses |
| [Wikipedia: DH parameters, modified DH section](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters) | Quick reference (linked from Lab 1) |
| [Franka robot specifications (DH table)](https://github.com/frankarobotics/docs/blob/master/source/robot_specifications.rst) | Ground truth for the FR3 parameters and joint limits |
| QUT Robot Academy, DH masterclass videos ([robotacademy.net.au](https://robotacademy.net.au/)) | Visual frame assignment |

**Deeper**

| Resource | Take from it |
|---|---|
| Spong, Hutchinson & Vidyasagar, *Robot Modeling and Control* (2nd ed., 2020), Ch. 3 | Standard DH, carefully, with many examples |
| Siciliano, Sciavicco, Villani & Oriolo, *Robotics: Modelling, Planning and Control* (Springer, 2009), Ch. 2 | Rigorous European-style treatment; the course's probable reference text |
| Corke, *Robotics, Vision and Control* (3rd ed., Python, 2023), Ch. 7 + [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) | `DHRobot`, `ERobot` (ETS) and a Panda model to test against |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Fast, exact FK from URDF. Your numerical ground truth from here on |
| Murray, Li & Sastry, Ch. 3 ([free](https://www.cds.caltech.edu/~murray/mlswiki/)) | PoE derivation and proofs |

## Warm-ups

1. **Prove the bridge.** Show that the MDH transform factors as
   $T_{\text{fixed}}(a_{i-1}, \alpha_{i-1}, d_i)\cdot R_z(\theta_i)$. Derive the fixed
   part as an `xyz` + `rpy` pair. Then check your formula against **every** row of
   `kinematics.yaml` by hand. Why are all `pitch` and `yaw` entries zero?
2. **Both conventions.** Write the standard DH table for the FR3 from the MDH one. Verify
   numerically that both give the same flange pose at 1,000 random configurations.
3. **Screw axes from the URDF.** Compute the FR3's $S_i$ and $M$ at its zero
   configuration, directly from `kinematics.yaml`, with no DH involved.
4. **Planar warm-up.** For a 3R planar arm, derive FK by hand three ways (geometry, DH,
   PoE) and confirm they match symbolically (SymPy).
5. **Workspace cloud.** Sample 10⁶ configurations within the FR3 limits, plot the flange
   positions (a slice through $y = 0$), and estimate the maximum reach. Overlay the 0.4 m
   point from Lab 2's `traj_waypoints.yaml`.

## Mini project: three-way FK cross-validation and a URDF → MDH/PoE extractor *(A: parts 1–2; B: part 3)*

Do this **after** submitting Lab 1, in your own `mamp_kinematics` package. Keep the
library separate from your lab code.

**Part 1: one robot, three FKs (C++, using `mamp_liegroups`).**
Implement `fk_mdh`, `fk_poe_space`, `fk_poe_body` and `fk_urdf_tree` (parse the URDF
with `urdfdom` or your own parser) for the FR3, with all intermediate link frames
available.

**Part 2: validation harness (Python, through bindings).** For 100,000 random
configurations within limits, compare all four against tf2 (via `robot_state_publisher`
on a bag or a live system), PyKDL and Pinocchio.

**Part 3: automatic extraction.** Given **any** serial-chain URDF (test on the FR3, a
UR5e, a KUKA iiwa and your Module 07 robot), compute:
(a) PoE screw axes and $M$; (b) an MDH table, with a fixed tool transform where needed.
Handle parallel and intersecting consecutive axes explicitly; for parallel axes the
common normal is not unique, so document your choice. Emit the table as Markdown and
YAML.

**Proof of competence**

* All FKs agree with Pinocchio to **≤ 1e-12 m and ≤ 1e-12 rad** over 100,000
  configurations, including every intermediate frame, not just the flange.
* The extractor reproduces Franka's published MDH table from the URDF alone, row for row
  (modulo documented sign or offset choices).
* Timing: single-configuration FK in under 1 µs in C++, and a batched version
  (≥ 10⁴ configurations per call) with throughput reported in configs/second. Compare it
  with Pinocchio.

**Stretch**

* Generate **symbolic** FK with SymPy or CasADi, emit C code, and compare speed with the
  hand-written version.
* Kinematic calibration study: perturb the URDF with small offsets, simulate noisy
  end-effector measurements, and recover the offsets by least squares on the PoE
  parameters. Report the observability of each parameter.

## Self-check

1. Write both DH conventions from memory and say where frame $i$ sits in each.
2. Why does Franka's MDH table have 8 rows for a 7-joint arm?
3. Prove that an MDH row is a fixed transform times $R_z(\theta_i)$. What does this say
   about the URDF origins?
4. Define a screw axis for a revolute joint. How do you get $B_i$ from $S_i$?
5. Give two practical advantages of PoE over DH.
6. What is the difference between reachable and dexterous workspace? How would you
   estimate each for the FR3?
7. Lab 2's mock robot starts at $q = 0$. Why is that a problem for the FR3, and what
   might it do to your IK?

Next: [Module 09: Differential kinematics](09-differential-kinematics.md)
