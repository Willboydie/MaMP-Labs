# Module 07: Robot description (URDF, xacro, SDF, MJCF)

> **Part II · Modelling maths** · Est. 12–16 h · Prerequisites: 05, 06 · Track A

## Why this matters / where it lives in this repo

The robot model is the single source of truth for kinematics, dynamics, collision
geometry, joint limits and hardware interfaces. Every tool reads it: tf2 via
`robot_state_publisher`, KDL in Lab 2, MoveIt, Gazebo and `ros2_control`.
[`franka_description`](../franka_description) is a professionally structured description
package and well worth reading in full:

| File | What it teaches |
|---|---|
| [`robots/fr3/fr3.urdf.xacro`](../franka_description/robots/fr3/fr3.urdf.xacro) | Top-level entry point: `xacro:arg`s, YAML loaded with `xacro.load_yaml`, one macro call |
| [`robots/common/franka_robot.xacro`](../franka_description/robots/common/franka_robot.xacro) | Composition: arm + optional end effector + Gazebo tags + `ros2_control`, all switched by `xacro:if` |
| [`robots/common/franka_arm.xacro`](../franka_description/robots/common/franka_arm.xacro) | The kinematic chain: 8 joints (7 revolute + fixed flange), links with optional self-collision capsules |
| [`robots/common/utils.xacro`](../franka_description/robots/common/utils.xacro) | Macros for inertials, link + self-collision sub-link, capsules, Gazebo joints |
| [`robots/fr3/*.yaml`](../franka_description/robots/fr3) | Kinematics, inertials, joint limits and dynamics as **data**, separate from structure |
| [`end_effectors/common/franka_hand.xacro`](../franka_description/end_effectors/common/franka_hand.xacro) | A `hand_tcp` virtual frame, prismatic fingers, a `mimic` joint |
| `meshes/.../visual/*.dae` vs `collision/*.stl` | High-detail visuals vs simplified collision meshes |
| [`urdfs/fr3.urdf`](../franka_description/urdfs/fr3.urdf) | A *generated* artefact, adapted for the course (mock `ros2_control` block). Generated files drift from their sources |
| [`test/urdf_tests.py`](../franka_description/test/urdf_tests.py) | Tests that run xacro with different mappings |

## Learning objectives

* Read and write URDF: links (visual, collision, inertial), joints (types, origin, axis,
  limits, dynamics, mimic) and the `ros2_control` and `gazebo` extension tags.
* Explain exactly how a joint's `origin` and `axis` define the parent → child
  transform as a function of $q$.
* Use xacro well: properties, macros (including `^` and `**block` parameters),
  conditionals, includes, YAML loading, and **lazy evaluation** with its pitfalls.
* Judge physical plausibility: mass, centre of mass, and **physically consistent inertia
  tensors**.
* Know URDF's limits (trees only, little physics) and how SDF and MJCF extend it.

## Key ideas to own

* **Joint semantics.** For a revolute joint,
  ${}^{parent}T_{child}(q) = T_{origin}\cdot \mathrm{Rot}(\text{axis}, q)$; for a
  prismatic joint, $T_{origin}\cdot \mathrm{Trans}(\text{axis}\cdot q)$. `origin`
  (`xyz`, then fixed-axis `rpy`) places the joint frame in the parent link frame. The
  child link frame **is** the joint frame. `axis` is expressed in the joint frame.
  Every Franka joint uses `axis="0 0 1"`, so all the geometry lives in the origins
  (Module 08 shows these origins are the modified DH parameters in disguise).
* **Links outnumber joints, and some links are empty.** A tree of $n$ links has $n-1$
  joints. On top of that, *fixed* joints add links with no degrees of freedom: a `base`
  or `world` anchor, a flange, a tool centre point (`fr3_hand_tcp`), sensor frames, and
  self-collision sub-links. Empty links are frames you want to name in tf2.
* **Geometry.** Visual meshes are detailed and textured (Collada `.dae`). Collision
  geometry is simplified (`.stl`, convex pieces or primitives) because collision checking
  dominates planning time (Modules 15–16). Franka also provides coarse **self-collision
  capsules** (`with_sc`), inflated by a `safety_distance`.
* **Inertials.** `<inertial>` gives mass, the COM pose and the inertia tensor *about the
  COM* in that frame. Physical consistency requires $I \succ 0$ **and** the triangle
  inequalities on the principal moments ($I_1 + I_2 \ge I_3$ and permutations). The
  strongest standard condition is positive-definiteness of the 4×4 **pseudo-inertia**
  matrix (Wensing et al.). Simulators misbehave with inconsistent inertias, and
  identified parameters (Module 11) must be projected onto this set.
* **Xacro essentials.**
  * `${expr}` evaluates Python-like expressions; `$(arg x)` reads arguments; `$(find pkg)`
    resolves package paths.
  * `xacro:macro` parameters: `p:=default`, `p:=^` (inherit from the enclosing scope),
    `p:=^|default`, and `**block` (insert XML with `xacro:insert_block`).
  * **Lazy evaluation:** properties are evaluated where they're *used*, and redefining a
    property overwrites it silently. Read `collision_capsule` in `utils.xacro` with that
    in mind (see the warm-ups).
  * `xacro.load_yaml` turns YAML into dot-accessible dictionaries. PyYAML silently
    accepts duplicate keys, keeping the last one.
* **Mesh URIs.** `package://franka_description/meshes/...` is resolved through the ament
  index. Non-ROS tools need absolute paths, which is why `scripts/create_urdf.py` has
  `--abs-path`.
* **Formats.**
  * *URDF:* trees only, minimal physics, no world.
  * *SDF (Gazebo):* closed loops, worlds, sensors, plugins and surface properties.
  * *MJCF (MuJoCo):* default classes, actuators, tendons, equality constraints and contact
    solver parameters. It can load a URDF with a `<mujoco>` extension tag but doesn't
    read `.dae`, so visual meshes need converting. Hand-tuned MJCF for the FR3 exists in
    MuJoCo Menagerie.
* **`robot_description` plumbing.** The URDF string is a parameter of
  `robot_state_publisher`, which also republishes it on the transient-local
  `/robot_description` topic that RViz and `ros2_control` read.
  `joint_state_publisher(_gui)` publishes values for every non-fixed, non-mimic joint.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [URDF tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html): visual model from scratch, movable model, physical and collision properties, xacro, `robot_state_publisher` | Hands-on URDF and xacro |
| [URDF XML specification](http://wiki.ros.org/urdf/XML) (ROS 1 wiki, still the reference) and [ros/urdfdom](https://github.com/ros/urdfdom) | Every tag and attribute. Read the `joint` and `link` pages closely |
| [xacro documentation](https://github.com/ros/xacro/wiki) | Macros, `^`, blocks, YAML, lazy evaluation |
| Articulated Robotics, URDF and xacro videos ([YouTube / articulatedrobotics.xyz](https://articulatedrobotics.xyz)) | A visual walkthrough of building a robot description |

**Deeper**

| Resource | Take from it |
|---|---|
| [SDFormat specification](http://sdformat.org/spec) | What SDF adds (worlds, loops, sensors) |
| [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) and [modelling guide](https://mujoco.readthedocs.io/en/stable/modeling.html) | MJCF concepts you'll need in Module 20 |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) `franka_fr3/` | A curated MJCF of the same robot. Compare it with the URDF |
| Wensing, Kim & Slotine, "Linear Matrix Inequalities for Physically Consistent Inertial Parameter Identification", IEEE RA-L 2018 | Pseudo-inertia and consistency conditions |
| [trimesh](https://trimesh.org/) | Mass properties from meshes (volume, COM, inertia at a given density) |
| [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) | How CAD models become URDF/SDF/MJCF in practice |

## Warm-ups

1. **Generate and diff.** Run `xacro` on `fr3.urdf.xacro` with several argument
   combinations (`hand:=true ee_id:=franka_hand`, `with_sc:=true`, `ros2_control:=true`)
   and diff each against `urdfs/fr3.urdf`. Explain every difference, including the
   `ros2_control` block.
2. **Draw the tree.** Use `urdf_to_graphviz` (from `liburdfdom-tools`, as is
   `check_urdf`) on the FR3 with the hand. Mark fixed joints, empty links and the mimic
   joint. Then answer Lab 1 step 1.7 *in your own words* in your notes.
3. **Bug hunt.** `collision_capsule` in `robots/common/utils.xacro` contains a latent
   lazy-evaluation bug that happens to be harmless for every capsule the Franka arms
   currently define. Find it, explain why it's harmless today, and say which capsule
   definition would expose it.
4. **Lint the data.** Run `yamllint` on `robots/*/inertials.yaml`. Explain the warning it
   gives for `fr3` and why xacro didn't complain.
5. **See what the planner sees.** In RViz, enable collision geometry on the `RobotModel`
   display, with and without `with_sc:=true`.
6. **Cross-format.** Load the FR3 URDF into MuJoCo (`python -m mujoco.viewer`) and into
   Gazebo (`gz sdf -p fr3.urdf > fr3.sdf`). List everything that breaks (mesh formats,
   `package://` paths) and fix it.

## Mini project: a dual-arm torso robot, described properly *(A: steps 1–4; B: the rest)*

Create `mamp_description` for a new robot: a torso on a **prismatic lift** and a
**yaw joint**, two FR3 arms at configurable shoulder poses with optional hands, a
pan-tilt head carrying a camera with a REP 103 **optical frame**, and a fixed table in a
`world` frame.

**Requirements**

1. All structure in xacro macros, all numbers in YAML (follow Franka's split of
   kinematics, inertials, limits and dynamics).
2. Reuse the Franka meshes and YAML. You'll find that `franka_arm` uses `arm_id`
   **both** to select the mesh/YAML model **and** as the name prefix, so two FR3s can't
   simply be called `left` and `right`. Design around this, e.g. with your own arm macro
   that separates `model` from `prefix`. Write down why your design is better.
3. Inertials for your own links computed from geometry (trimesh at a stated density),
   not invented.
4. A mock `ros2_control` block for every actuated joint, so `mamp_bringup` (Module 03)
   can launch it.
5. A **validator** (pytest, in the style of `franka_description/test/urdf_tests.py`)
   that checks: positive masses; $I \succ 0$ and the triangle inequalities;
   pseudo-inertia PD; COM inside the link's collision-mesh bounding box (with tolerance);
   `lower < upper` limits; unit-norm axes; every mesh URI resolves; unique names; a
   connected tree; and that `xacro` succeeds for every argument combination.
6. Run the validator on **`franka_description` itself** (all three arms: `fr3`, `fp3`,
   `fer`) and report the results. At the time of writing, at least one shipped link
   fails a basic physical-consistency check.
7. Export to SDF and MJCF and load both in Gazebo and MuJoCo.

**Proof of competence**

* Your robot passes every validator check. Every check has a test fixture that makes it
  fail.
* A validator report on `franka_description` that names the inconsistent link, gives
  the violated inequality numerically, and proposes a minimal correction (nearest
  consistent tensor, e.g. by projection onto the feasible set).
* Total mass and whole-body COM agree across URDF, SDF and MJCF to 1e-9 at three
  configurations.
* RViz with `joint_state_publisher_gui` moves all 2×7 + 2 + 2 joints, and TF shows the
  camera optical frame correctly (z forward).

**Stretch**

* A per-joint calibration-offset mechanism (xacro args loaded from a calibration YAML),
  with a test that zero offsets reproduce the nominal model exactly.
* Your own URDF → MJCF converter for this robot (mesh conversion, actuators, default
  classes). Compare it with MuJoCo's built-in URDF import.

## Self-check

1. Where is a child link's frame relative to its joint? In which frame is `axis`
   expressed?
2. Give three reasons a URDF contains links with no geometry.
3. Who consumes visual, collision and self-collision geometry respectively?
4. What makes an inertia tensor physically consistent, and why is positive-definiteness
   not enough?
5. What does `p:=^|default` mean in a xacro macro? Describe a bug caused by lazy
   evaluation.
6. Why can't URDF describe a parallel mechanism, and what would you use instead?
7. The checked-in `urdfs/fr3.urdf` and the xacro sources disagree. Which is the source of
   truth, and how would you stop them drifting?

Next: [Module 08: Forward kinematics](08-forward-kinematics.md)
