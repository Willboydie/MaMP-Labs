# Master resource list

Everything the modules cite, in one place, plus a few general references. "Free" means
the authors provide a legitimate free copy. Links were current when written
(October 2026). If one moves, search for the title.

## The four books to keep on your desk

| Book | Why | Modules |
|---|---|---|
| K. Lynch & F. Park, *Modern Robotics: Mechanics, Planning, and Control* (CUP, 2017). **Free** PDF + short videos at [the book's wiki](https://hades.mech.northwestern.edu/index.php/Modern_Robotics); also a Coursera specialisation | Modern, Lie-group-based treatment of nearly everything in Parts II–VI | 05, 08–14, 16, 18 |
| B. Siciliano, L. Sciavicco, L. Villani & G. Oriolo, *Robotics: Modelling, Planning and Control* (Springer, 2009) | The classic European modelling/planning/control text: rigorous and complete | 08–13 |
| S. M. LaValle, *Planning Algorithms* (CUP, 2006). **Free** at [lavalle.pl/planning](https://lavalle.pl/planning/) | The motion-planning bible | 14–18 |
| F. Martín Rico, *A Concise Introduction to Robot Programming with ROS 2* (CRC Press), with code at [fmrico/book_ros2](https://github.com/fmrico/book_ros2) | C++-first ROS 2, written by a ROS 2 core contributor | 01–03 |

## Specialist books

| Book | Use it for |
|---|---|
| J. Craig, *Introduction to Robotics: Mechanics and Control* (4th ed., 2017) | Modified DH (the Franka convention), frames notation |
| M. Spong, S. Hutchinson & M. Vidyasagar, *Robot Modeling and Control* (2nd ed., 2020) | Standard DH, velocity kinematics, control |
| P. Corke, *Robotics, Vision and Control* (3rd ed., Springer, 2023; Python edition) | Hands-on spatial maths and kinematics with code |
| R. Murray, Z. Li & S. Sastry, *A Mathematical Introduction to Robotic Manipulation* (1994). **Free** at [the book's wiki](https://www.cds.caltech.edu/~murray/mlswiki/) | Rigorous screw theory and grasping |
| R. Featherstone, *Rigid Body Dynamics Algorithms* (Springer, 2008) + [spatial notes](http://royfeatherstone.org/spatial/) | Spatial algebra, RNEA/CRBA/ABA |
| H. Choset et al., *Principles of Robot Motion* (MIT Press, 2005) | Approachable motion-planning text |
| J.-C. Latombe, *Robot Motion Planning* (1991) | Classic exact planning |
| C. Ericson, *Real-Time Collision Detection* (2004) | Collision detection in practice |
| L. Biagiotti & C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008) | Every motion profile and spline |
| J. Nocedal & S. Wright, *Numerical Optimization* (2nd ed., 2006) | The optimisation methods behind Modules 10 and 17 |
| S. Boyd & L. Vandenberghe, *Convex Optimization*. **Free** at [stanford.edu/~boyd/cvxbook](https://web.stanford.edu/~boyd/cvxbook/) | Convexity, duality, QPs |
| J. Rawlings, D. Mayne & M. Diehl, *Model Predictive Control* (2nd ed.). **Free** at [the book's site](https://sites.engineering.ucsb.edu/~jbraw/mpc/) | MPC theory |
| T. Barfoot, *State Estimation for Robotics* (2nd ed., 2024). **Free** copy on [the author's page](https://asrl.utias.utoronto.ca/~tdb/) | Matrix Lie groups and estimation |
| K. Åström & R. Murray, *Feedback Systems*. **Free** at [fbswiki.org](https://fbswiki.org/) | Control fundamentals |
| J.-J. Slotine & W. Li, *Applied Nonlinear Control* (1991) | Lyapunov and adaptive manipulator control |
| W. Khalil & E. Dombre, *Modeling, Identification and Control of Robots* (2002) | Dynamic identification |
| C. Ott, *Cartesian Impedance Control of Redundant and Flexible-Joint Robots* (2008) | Franka/DLR-style impedance control |
| R. Sutton & A. Barto, *Reinforcement Learning: An Introduction* (2nd ed.). **Free** at [incompleteideas.net](http://incompleteideas.net/book/the-book-2nd.html) | RL foundations |
| B. Siciliano & O. Khatib (eds.), *Springer Handbook of Robotics* (2nd ed., 2016) | An authoritative chapter on almost any topic |

## Courses and lecture series

| Course | Notes |
|---|---|
| *Modern Robotics* (Northwestern; videos on the book wiki and on Coursera) | Short videos matching every chapter |
| R. Tedrake, **Robotic Manipulation** (MIT 6.421) at [manipulation.mit.edu](https://manipulation.mit.edu/) | Perception, planning and control with Drake notebooks; lectures on YouTube |
| R. Tedrake, **Underactuated Robotics** (MIT 6.832) at [underactuated.mit.edu](https://underactuated.mit.edu/) | Dynamics, optimal control, trajectory optimisation, MPC; lectures on YouTube |
| O. Khatib, **Stanford CS223A** at [see.stanford.edu/Course/CS223A](https://see.stanford.edu/Course/CS223A) | Classic kinematics, dynamics and control lectures |
| P. Corke, **QUT Robot Academy** at [robotacademy.net.au](https://robotacademy.net.au/) | Bite-size spatial maths and kinematics lessons |
| S. Levine, **Berkeley CS 285** at [rail.eecs.berkeley.edu/deeprlcourse](https://rail.eecs.berkeley.edu/deeprlcourse/) | Deep RL and imitation learning |
| M. Likhachev, **CMU 16-782** *Planning and Decision-making in Robotics* (slides) | Search-based planning |
| [MIT Missing Semester](https://missing.csail.mit.edu/) | Shell, git and tooling fluency |
| M. Kelly, *Introduction to Trajectory Optimization* (SIAM Review 2017 + video) | Direct collocation, step by step |
| Brian Douglas, *Control Systems Lectures* / MATLAB Tech Talks (YouTube) | Control intuition |

## Official documentation

| Docs | Modules |
|---|---|
| [ROS 2 Humble docs](https://docs.ros.org/en/humble/) · [ROS 2 design articles](https://design.ros2.org/) · [REPs](https://www.ros.org/reps/rep-0000.html) (103, 105, 149) | 00–07 |
| [ros2_control](https://control.ros.org/humble/index.html) · [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) | 12, 20 |
| [MoveIt 2](https://moveit.picknik.ai/main/index.html) | 19 |
| [OMPL](https://ompl.kavrakilab.org/) · [Planner Arena](https://plannerarena.org/) | 16, 18 |
| [Gazebo](https://gazebosim.org/docs) · [MuJoCo](https://mujoco.readthedocs.io/) · [Drake](https://drake.mit.edu/) · [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) | 20 |
| [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) | 08–11, 17 |
| [Nav2](https://docs.nav2.org/) | 18 |
| [Franka documentation](https://frankaemika.github.io/docs/) · [Franka robot specifications](https://github.com/frankarobotics/docs/blob/master/source/robot_specifications.rst) · [franka_ros2](https://github.com/frankarobotics/franka_ros2) | All, for this robot |

## Practical video and blog channels

* **Articulated Robotics** (YouTube, [articulatedrobotics.xyz](https://articulatedrobotics.xyz/)): ROS 2, URDF, `ros2_control` and Gazebo for beginners.
* **[Robotics Back-End](https://roboticsbackend.com/)**: ROS 2 tutorials.
* **ROSCon** talk archive at [roscon.ros.org](https://roscon.ros.org/): design talks from the people who build ROS 2, MoveIt and `ros2_control`.
* [Ben Eater & 3Blue1Brown, interactive quaternions](https://eater.net/quaternions).
* Casey Muratori, "Implementing GJK" (2006, YouTube).

## Libraries and tools

| Library | What for | Module |
|---|---|---|
| rclcpp / rclpy, tf2, robot_state_publisher, xacro | ROS 2 core | 01–07 |
| [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Foxglove](https://foxglove.dev/), [Rerun](https://rerun.io/), [MCAP](https://mcap.dev/) | Visualisation and logging | 04 |
| [Sophus](https://github.com/strasdat/Sophus), [manif](https://github.com/artivis/manif), [spatialmath-python](https://pypi.org/project/spatialmath-python/) | Lie groups and spatial maths | 05 |
| [orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics), [Pinocchio](https://github.com/stack-of-tasks/pinocchio), [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) | Kinematics and dynamics | 08–11 |
| [pick_ik](https://github.com/PickNikRobotics/pick_ik), [Pink](https://github.com/stephane-caron/pink), [mink](https://github.com/kevinzakka/mink), [frankik](https://pypi.org/project/frankik/), [frantik](https://github.com/CoMMALab/frantik), [qpsolvers](https://github.com/qpsolvers/qpsolvers) | Inverse kinematics | 10 |
| [ros2_controllers](https://github.com/ros-controls/ros2_controllers), [realtime_tools](https://github.com/ros-controls/realtime_tools) | Control | 12 |
| [toppra](https://github.com/hungpham2511/toppra), [Ruckig](https://github.com/pantor/ruckig) | Time parameterisation | 13 |
| [FCL](https://github.com/flexible-collision-library/fcl), [Coal](https://github.com/coal-library/coal), [CoACD](https://github.com/SarahWeiii/CoACD), [trimesh](https://trimesh.org/) | Collision and geometry | 07, 15 |
| [OMPL](https://ompl.kavrakilab.org/), [VAMP](https://github.com/kavrakilab/vamp), [cuRobo](https://curobo.org/) | Sampling-based and GPU planning | 16 |
| [CasADi](https://web.casadi.org/), [acados](https://docs.acados.org/), [OSQP](https://osqp.org/), IPOPT, [Crocoddyl](https://github.com/loco-3d/crocoddyl), [Drake](https://drake.mit.edu/) | Optimisation and optimal control | 17 |
| [Tesseract](https://github.com/tesseract-robotics/tesseract), [descartes_light](https://github.com/swri-robotics/descartes_light), [PDDLStream](https://github.com/caelan/pddlstream) | Process and task planning | 18 |
| [MoveIt 2](https://github.com/moveit/moveit2), [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor) | Manipulation framework | 19 |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), gz_ros2_control, mujoco_ros2_control | Simulation | 20 |
| [LeRobot](https://github.com/huggingface/lerobot), [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3), [CleanRL](https://github.com/vwxyzjn/cleanrl), [MuJoCo Playground](https://playground.mujoco.org/) | Learning | 21 |

## Twenty-five papers worth reading even if you read nothing else

1. Macenski et al., "Robot Operating System 2: Design, Architecture, and Uses in the Wild", *Science Robotics* 2022.
2. Solà, Deray & Atchuthan, "A micro Lie theory for state estimation in robotics", arXiv 2018.
3. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg–Marquardt Method", IEEE T-RO 2011.
4. Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots", IJRR 1986.
5. Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation", IEEE JRA 1987.
6. Hogan, "Impedance Control: An Approach to Manipulation", ASME JDSMC 1985.
7. Gaz et al., "Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization", RA-L 2019.
8. Pham & Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis", IEEE T-RO 2018.
9. Berscheid & Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States", RSS 2021.
10. Kavraki, Švestka, Latombe & Overmars, "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces", IEEE TRA 1996.
11. Kuffner & LaValle, "RRT-Connect: An Efficient Approach to Single-Query Path Planning", ICRA 2000.
12. Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning", IJRR 2011.
13. Gammell, Barfoot & Srinivasa, "Batch Informed Trees (BIT\*)", IJRR 2020.
14. Ratliff, Zucker, Bagnell & Srinivasa, "CHOMP: Gradient Optimization Techniques for Efficient Motion Planning", ICRA 2009.
15. Schulman et al., "Motion Planning with Sequential Convex Optimization and Convex Collision Checking", IJRR 2014.
16. Kalakrishnan et al., "STOMP: Stochastic Trajectory Optimization for Motion Planning", ICRA 2011.
17. Marcucci et al., "Motion Planning around Obstacles with Convex Optimization", *Science Robotics* 2023.
18. Berenson, Srinivasa & Kuffner, "Task Space Regions", IJRR 2011.
19. Kingston, Moll & Kavraki, "Sampling-Based Methods for Motion Planning with Constraints", *Annu. Rev.* 2018.
20. Garrett et al., "Integrated Task and Motion Planning", *Annu. Rev.* 2021.
21. Thomason, Kingston & Kavraki, "Motions in Microseconds via Vectorized Sampling-Based Planning", ICRA 2024.
22. Sundaralingam et al., "cuRobo: Parallelized Collision-Free Robot Motion Generation", ICRA 2023.
23. Todorov, Erez & Tassa, "MuJoCo: A Physics Engine for Model-Based Control", IROS 2012.
24. Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion", RSS 2023.
25. Fishman et al., "Motion Policy Networks", CoRL 2022.

## Communities, venues and staying current

* **[ROS Discourse](https://discourse.openrobotics.org/)**: release announcements and the weekly "ROS News".
* **[Robotics Stack Exchange](https://robotics.stackexchange.com/)**: Q&A (the successor to ROS Answers).
* Package GitHub issue trackers: search them before asking anywhere else.
* **Conferences:** ICRA, IROS, RSS, CoRL, Humanoids, ROSCon. **Journals:** IJRR, IEEE T-RO, IEEE RA-L, *Science Robotics*, *Annual Review of Control, Robotics, and Autonomous Systems*.

Back to the [roadmap](README.md).
