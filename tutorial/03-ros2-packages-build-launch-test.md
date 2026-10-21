# Module 03: Packages, build, launch, parameters and testing

> **Part I · ROS 2 foundations** · Est. 12–16 h · Prerequisites: 01 · Track A

## Why this matters / where it lives in this repo

The repo contains both package build types and two different launch-file styles. All
of them are worth reading line by line:

| File | What to learn from it |
|---|---|
| [`franka_description/CMakeLists.txt`](../franka_description/CMakeLists.txt) | `ament_cmake` package that installs data directories (`install(DIRECTORY ...)`) and registers pytest tests |
| [`Lab1/transform_helpers/setup.py`](../Lab1/transform_helpers/setup.py) + `setup.cfg` | `ament_python`: the ament-index marker, `data_files` for launch/config, `console_scripts` entry points, script install directory |
| [`Lab2/trajectory_demo/package.xml`](../Lab2/trajectory_demo/package.xml) | A `package.xml` that declares **no runtime dependencies**, so `rosdep` can't help you. Compare it with `transform_helpers/package.xml` |
| [`Lab1/.../bringup.launch.py`](../Lab1/transform_helpers/launch/bringup.launch.py) | Declarative launch: `DeclareLaunchArgument`, `LaunchConfiguration`, `FindPackageShare`, `PathJoinSubstitution`, `Command(['xacro ', ...])`, `ParameterValue(..., value_type=str)` |
| [`Lab2/.../bringup.launch.py`](../Lab2/trajectory_demo/launch/bringup.launch.py) | Adds `ros2_control_node`, controller `spawner`s, a static TF and a **remapping**. Mixes launch-time substitutions with `get_package_share_directory` evaluated at description time |
| [`franka_description/launch/visualize_franka.launch.py`](../franka_description/launch/visualize_franka.launch.py) | Imperative style: `OpaqueFunction` + `context.perform_substitution` + the `xacro` Python API |
| [`franka_description/test/urdf_tests.py`](../franka_description/test/urdf_tests.py) | Testing a *robot description* with plain pytest |

## Learning objectives

* Create `ament_cmake`, `ament_python` and mixed C++/Python packages with correct
  `package.xml` dependencies (REP 149).
* Write launch files that are parameterised, composable, namespaced and conditional, and
  know exactly **when** each substitution is evaluated.
* Manage parameters with YAML files, overrides and wildcards.
* Test at three levels: **unit** (pytest/gtest), **lint** (ament_lint) and
  **integration** (`launch_testing`).
* Structure code so the algorithms are ROS-free libraries and the nodes are thin
  wrappers.

## Key ideas to own

* **`package.xml` dependency tags:** `buildtool_depend` (ament_cmake), `build_depend`
  (headers and CMake config needed to compile), `exec_depend` (needed at run time,
  including Python imports), `depend` (both), `test_depend`. `rosdep` and the build farm
  only see what's declared.
* **What `ros2 launch pkg file.launch.py` needs from an `ament_python` package:** the ament
  index marker (`resource/<pkg>` installed to `share/ament_index/...`), `package.xml` in
  `share/<pkg>`, the launch file installed to `share/<pkg>/launch`, and for `ros2 run`
  the entry point installed into `lib/<pkg>`. That last part is what `setup.cfg`'s
  `script_dir`/`install_scripts` lines do.
* **`ament_cmake` essentials:** `find_package(...)`, `add_library` / `add_executable`,
  link with `ament_target_dependencies` (common on Humble; newer distros prefer
  `target_link_libraries` with exported targets), `install(TARGETS ... DESTINATION
  lib/${PROJECT_NAME})` for executables, `install(DIRECTORY launch config ... DESTINATION
  share/${PROJECT_NAME})` for data, and `ament_package()` last.
* **Launch is a two-phase system.** First a *description* is built (your Python runs).
  Then it is *executed*, and substitutions (`LaunchConfiguration`, `FindPackageShare`,
  `Command`, `PathJoinSubstitution`) are resolved. So:
  * `get_package_share_directory("x")` runs **when the description is built** and can't
    depend on a launch argument.
  * `FindPackageShare(LaunchConfiguration("pkg"))` resolves **at launch time**.
  * When you need real Python logic over argument values (e.g. calling the `xacro` API),
    use an `OpaqueFunction`, as `visualize_franka.launch.py` does.
* **`ParameterValue(Command([...]), value_type=str)`.** Without it, the launch system
  tries to parse the URDF text produced by `xacro` as YAML and fails or mangles it.
* **Composition tools:** `GroupAction`, `PushRosNamespace`, `IncludeLaunchDescription`,
  `IfCondition`/`UnlessCondition`, and `RegisterEventHandler(OnProcessExit(...))` to
  sequence start-up (e.g. spawn the arm controller only after the joint state broadcaster
  is up; the `ros2_control_demos` launch files use this).
* **Parameters from YAML:** `node_name: {ros__parameters: {...}}`; `/**:` matches every
  node. Use `ros2 param dump /node` to snapshot a running configuration.
* **Architecture rule: libraries first.** Put FK, IK and planners in a ROS-free library
  (`mamp_kinematics/core`) with its own unit tests, then wrap it in a node. You can then
  test the maths in milliseconds without starting DDS, reuse it offline in notebooks, and
  bind C++ into Python with pybind11 or nanobind.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) | Both build types |
| [ament_cmake user documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html) and [ament_cmake_python](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html) | Exporting targets, mixed C++/Python packages |
| [Launch tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html): substitutions, event handlers, [managing large projects](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html) | Everything in the "Key ideas" launch bullets |
| [Testing tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html) and [launch_testing integration tests](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html) | `colcon test`, gtest, pytest, launch_testing |
| [REP 149: package format 3](https://www.ros.org/reps/rep-0149.html) | Authoritative meaning of each tag |

**Deeper**

| Resource | Take from it |
|---|---|
| [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/) | Target-based CMake; you'll write a lot of it |
| [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) launch files | Idiomatic bring-up sequencing with event handlers |
| [pybind11 docs](https://pybind11.readthedocs.io/) | Exposing your C++ libraries to Python (used from Module 05 on) |

## Warm-ups

1. **Fix the manifest.** In your fork, give `trajectory_demo/package.xml` a complete set of
   `exec_depend`s. Prove it in a clean container from Module 00: `rosdep install` then
   `ros2 run trajectory_demo traj_demo` gets past all imports, except the external
   `youbot_kinematics`, which you should document.
2. **Remove `ParameterValue`** from a copy of Lab 1's launch file. Read the error, and
   explain it in one sentence.
3. **Evaluation time.** Make a copy of Lab 2's launch whose controller YAML package is a
   launch argument. Show that `get_package_share_directory` can't support that, then make
   it work with substitutions.
4. **Three formats.** Rewrite Lab 1's launch in XML and YAML launch formats. Which is
   easiest to read, and what can only Python do?
5. **Library + node + tests (C++).** Create `mamp_kinematics` with a ROS-free library
   target (`so3.hpp/.cpp`), a node that uses it, a gtest for the library and a
   `launch_testing` smoke test for the node. Run `colcon test` and
   `colcon test-result --verbose`.

## Mini project: `mamp_bringup`, one parameterised launch for everything *(A)*

Write a bring-up package you will extend for the rest of the series.

**Requirements**

1. Launch arguments, each validated with a clear error for bad values (`choices=`):
   `arm_id` (fr3/fp3/fer), `ee_id` (none/franka_hand/cobot_pump), `n_robots` (1–4),
   `layout` (YAML file of base poses), `hardware` (mock now, `gazebo`/`mujoco` placeholders
   for Module 20), `rviz` and `gui` booleans.
2. Generate each robot's description by running `xacro` on
   `franka_description/robots/<arm_id>/<arm_id>.urdf.xacro` with mappings. Do not use
   the pre-generated `urdfs/fr3.urdf`.
3. Each robot gets its own namespace, `robot_state_publisher` with a `frame_prefix`, a
   controller manager and controllers, all attached to a common `world` frame.
4. Controller spawning is **sequenced** with event handlers, never with sleeps.
5. Write your own small `ros2_control` xacro macro for mock hardware. The upstream macro in
   [`franka_arm.ros2_control.xacro`](../franka_description/robots/common/franka_arm.ros2_control.xacro)
   references the older `fake_components/GenericSystem` plugin name, while the course's
   generated `urdfs/fr3.urdf` uses `mock_components/GenericSystem`. Find out which your
   distro provides.
6. A `launch_testing` integration test that brings up `n_robots:=2` headless and asserts:
   both `/robotN/joint_states` topics publish; TF resolves `world → robotN/fr3_link8` for
   both; and a `FollowJointTrajectory` goal to each robot succeeds and moves only that
   robot.

**Proof of competence**

* `ros2 launch mamp_bringup bringup.launch.py n_robots:=2 arm_id:=fr3` shows two arms in
  RViz, and `ros2 launch ... --show-args` documents every argument.
* `colcon test --packages-select mamp_bringup` passes in CI (Module 00 container).
  Breaking a frame name or a controller name makes it fail.

**Stretch**

* Generate the RViz config programmatically for N robots.
* Accept a whole "scenario" YAML (robots, poses, controllers, objects) and validate it
  with a JSON Schema before launching.

## Self-check

1. List everything an `ament_python` package must install for `ros2 launch` and `ros2 run`
   to work.
2. `depend` vs `build_depend` vs `exec_depend`: which one would you use for `numpy`, for a
   C++ header-only library, for `rclcpp`?
3. Why does `ParameterValue(..., value_type=str)` exist?
4. When is a `LaunchConfiguration` resolved? When would you reach for `OpaqueFunction`?
5. How do you start the arm controller only after the joint state broadcaster is active,
   without `sleep`?
6. Give three concrete benefits of keeping your IK solver out of the ROS node class.

Next: [Module 04: Tooling, visualisation and debugging](04-ros2-tooling-visualisation-debugging.md)
