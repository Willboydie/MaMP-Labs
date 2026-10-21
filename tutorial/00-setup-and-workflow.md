# Module 00: Setup and professional workflow

> **Part 0 · Orientation** · Est. 8–12 h · Prerequisites: none · Track A

## Why this matters / where it lives in this repo

Every lab README begins with environment set-up (`rosdep`, `colcon build`,
`source install/setup.bash`), and most early ROS 2 frustration comes from the environment
rather than the code. In this repo:

* [`Lab1/env.yml`](../Lab1/env.yml) is a **RoboStack** (conda) environment: ROS 2 Humble
  from conda-forge, which also works on macOS and Windows.
* [`Lab2/README.md`](../Lab2/README.md) installs `ros2_control`, KDL and controller
  packages with `apt`.
* [`Lab2/trajectory_demo/trajectory_demo/traj_demo.py`](../Lab2/trajectory_demo/trajectory_demo/traj_demo.py)
  imports `youbot_kinematics.urdf` and `youbot_kinematics.urdf_parser`. **That package
  is not in this repository.** It comes from the course's main repo or materials. Its
  role is to parse a URDF (the `urdf_parser_py` job) and convert it into a `PyKDL.Tree`
  (the `kdl_parser_py` job). If the import fails, that is why.
* [`franka_description`](../franka_description) is an `ament_cmake` package; the lab
  packages are `ament_python`. You will build both kinds.

## Learning objectives

* Install ROS 2 Humble via at least one route (native, Docker, RoboStack or WSL2) and
  explain the trade-offs.
* Explain what `source /opt/ros/humble/setup.bash` actually does to your environment.
* Explain workspaces, **underlays and overlays**, and the `src/ build/ install/ log/`
  layout.
* Use `colcon` and `rosdep` fluently, and know their most useful flags.
* Run GUIs (RViz2, `joint_state_publisher_gui`) from a container.
* Keep your lab work in git with an upstream remote for course updates.

## Key ideas to own

* **ROS 2 is a set of libraries and tools installed into a prefix** (`/opt/ros/humble`).
  Sourcing `setup.bash` prepends that prefix to `AMENT_PREFIX_PATH`,
  `CMAKE_PREFIX_PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH` and `PATH`, and sets `ROS_DISTRO`
  and related variables. There is no daemon you "start". The `ros2 daemon` is only a
  discovery cache for the CLI.
* **Overlay:** your workspace's `install/` is another prefix layered on top of the
  underlay. `install/setup.bash` sources the underlays it was built against *and* the
  workspace; `install/local_setup.bash` sources only this workspace. Packages in an
  overlay shadow same-named packages in the underlay.
* **The ament index.** `ros2 run`, `ros2 launch`, `FindPackageShare` and
  `get_package_share_directory` find packages through marker files in
  `share/ament_index/resource_index/packages/`. That is why every `ament_python`
  package has a `resource/<pkg>` file and a `data_files` entry for it in `setup.py`.
* **Install space vs source space.** Launch files, URDFs and RViz configs are found in
  `install/<pkg>/share/<pkg>/...`, **not** in `src/`. If you edit a YAML file in `src/`
  and nothing changes, you probably forgot to rebuild, or you aren't using
  `--symlink-install`.
* **`colcon` flags you will use daily:**
  `--symlink-install`, `--packages-select <p>`, `--packages-up-to <p>`,
  `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`,
  `--event-handlers console_direct+`, then `colcon test` and
  `colcon test-result --verbose`.
* **`rosdep`** maps the dependency keys in `package.xml` to system packages (apt, pip,
  conda). It only knows what `package.xml` declares. A package that forgets
  `<exec_depend>` (look at `Lab2/trajectory_demo/package.xml`) will build and then fail
  at import time.
* **Network isolation:** `ROS_DOMAIN_ID` (0–101 is safe) separates DDS traffic.
  On Humble, `ROS_LOCALHOST_ONLY=1` keeps traffic on your machine. Jazzy and later use
  `ROS_AUTOMATIC_DISCOVERY_RANGE` instead. On a shared university network,
  classmates' robots *will* show up in your `ros2 topic list` unless you isolate.
* **Middleware choice:** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (or `rmw_fastrtps_cpp`,
  `rmw_zenoh_cpp` on Jazzy and later) switches the DDS/transport layer without code
  changes. Module 01 explains why you might care.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| [ROS 2 Humble installation](https://docs.ros.org/en/humble/Installation.html) | Native Ubuntu 22.04 install (preferred) |
| [Configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) | Sourcing, `ROS_DOMAIN_ID`, localhost-only |
| [Using colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and [Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) | Underlay/overlay, build flags |
| [Managing dependencies with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) | How keys resolve; `rosdep check` |
| [RoboStack](https://robostack.github.io/) | Conda route (what `Lab1/env.yml` uses) for macOS/Windows |

**Deeper**

| Resource | Take from it |
|---|---|
| [OSRF ROS Docker images](https://hub.docker.com/r/osrf/ros) and the [ROS 2 Docker how-to](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html) | Base images and running nodes across containers |
| [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers) + [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace) | A ready ROS 2 dev-container template to study and adapt |
| Articulated Robotics, "Docker for Robotics" series (YouTube, [articulatedrobotics.xyz](https://articulatedrobotics.xyz)) | GUI forwarding, devices, networking in containers |
| [ros-tooling/setup-ros](https://github.com/ros-tooling/setup-ros) + [ros-tooling/action-ros-ci](https://github.com/ros-tooling/action-ros-ci) | GitHub Actions CI for ROS 2 |
| [MIT "Missing Semester"](https://missing.csail.mit.edu/) | Shell, tmux, git and debugging fluency (skim what you already know) |

## Warm-ups

1. **Cross-language hello.** Run `ros2 run demo_nodes_cpp talker` in one terminal and
   `ros2 run demo_nodes_py listener` in another. A C++ process and a Python process are
   now talking over DDS. Then set different `ROS_DOMAIN_ID`s in the two terminals and
   watch it stop.
2. **Anatomy of an install.** Build this repo in `~/ros_ws/src`. Find where
   `bringup.launch.py`, `ros2_controllers.yaml` and `fr3.urdf` ended up under
   `install/`. Trace each one back to the line in `setup.py` or `CMakeLists.txt` that put
   it there.
3. **Environment diff.** Run `env | sort > before.txt`, source the workspace, run
   `env | sort > after.txt`, then `diff before.txt after.txt`. Explain every changed
   variable.
4. **Dependency audit.** Run `rosdep check --from-paths src --ignore-src -r`. Then read
   `traj_demo.py`'s imports and list every `<exec_depend>` that
   `trajectory_demo/package.xml` *should* declare. You will fix it properly in Module 03.
5. **Symlink experiment.** Edit `Lab2/trajectory_demo/cfg/traj_waypoints.yaml`. Does the
   node see the change with and without `--symlink-install`? Why?

## Mini project: one-command lab environment *(A)*

Build a reproducible, containerised development environment for this repository that a
classmate could use on a fresh laptop.

**Requirements**

1. A `Dockerfile` based on `ros:humble` (or `osrf/ros:humble-desktop`) that installs
   every dependency of `franka_description`, `transform_helpers` and `trajectory_demo`,
   via `rosdep` where possible. Run as a non-root user with your UID.
2. GUI support: X11 forwarding on Linux, plus one fallback that works on macOS/Windows
   (e.g. a noVNC/web desktop image, or WSLg).
3. A `.devcontainer/devcontainer.json` so VS Code opens the repo inside the container with
   IntelliSense working for both Python (`rclpy`) and C++ (`compile_commands.json`).
4. A `Makefile` (or `justfile`) with `build`, `test`, `lab1`, `lab2` and `clean`
   targets.
5. A GitHub Actions workflow that builds the workspace and runs `colcon test`. The
   lint tests in the lab packages (`test_flake8.py`, `test_pep257.py`) will fail on
   starter code. Decide deliberately how CI treats them and document why.
6. A **headless smoke test**: a script or `launch_testing` test that starts
   `robot_state_publisher` with `fr3.urdf` (plus a dummy joint-state publisher) and
   asserts that the transform `base → fr3_link8` is available within 5 seconds.

**Proof of competence**

* A fresh `git clone` followed by `make lab1` shows RViz with the FR3 within 10 minutes
  on a clean machine (time it, with a cold Docker cache).
* CI is green, with a status badge in your fork's README.
* The smoke test fails when you deliberately break the URDF (e.g. rename `fr3_link8`)
  and passes when you restore it. A test that can't fail proves nothing.

**Stretch**

* A Jazzy image alongside Humble, with a CI matrix over both.
* Multi-arch builds (amd64 + arm64 for Apple Silicon) with `docker buildx`.
* `pre-commit` hooks: `ruff`/`flake8`, `clang-format`, and XML well-formedness checks
  for URDF/xacro files.
* `ccache` in the container, with the cache persisted in a volume. Measure the
  rebuild-time improvement.

## Self-check

1. What is the difference between `install/setup.bash` and `install/local_setup.bash`?
2. You added a new `console_scripts` entry point to `setup.py` and `ros2 run` says "No
   executable found". Why, and what fixes it?
3. What does `--symlink-install` change for an `ament_python` package, and for an
   `ament_cmake` one?
4. Two students on the same Wi-Fi both run Lab 2. One student's robot occasionally moves
   when the other publishes a trajectory. Explain why, and give two fixes.
5. `rosdep install` for `trajectory_demo` reports success, yet the node crashes with
   `ModuleNotFoundError: PyKDL`. Explain the chain of causes.

Next: [Module 01: The ROS 2 graph](01-ros2-graph-and-communication.md)
