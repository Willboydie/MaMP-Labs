# Module 21: Robot learning for motion

> **Part VIII · Learning** · Est. 25–40 h · Prerequisites: 17, 20 (plus basic deep learning: PyTorch or JAX) · Track B

## Why this matters

Your degree is in Robotics **and AI**. The current frontier of manipulation mixes
classical modelling and planning (everything so far) with learned components: policies
trained from demonstrations, reinforcement learning in massively parallel simulation,
generative models of motion, and large vision-language-action models. Someone who
understands *both* sides can tell when learning genuinely helps (contact-rich skills,
perception-heavy tasks, fast amortised planning) and when it is an expensive way to
re-derive inverse kinematics. This module gives you that judgement, built on the
planners and simulators you've already written.

## Learning objectives

* Place learning in the motion stack: learned planner components, learned policies,
  learned models, and foundation models.
* Implement **behaviour cloning** with modern action representations (action chunking,
  diffusion/flow policies), and explain distribution shift and DAgger.
* Train **RL** policies (PPO/SAC) in parallel simulation, design rewards, and know when
  residual RL on a classical controller is the better choice.
* Use learning to **accelerate classical planning**: learned samplers, seeds and
  distilled planners.
* Evaluate learned systems honestly against classical baselines.

## Key ideas to own

* **Behaviour cloning (BC)** is supervised learning from observations to actions. Small
  errors take you to states the expert never visited, and errors compound
  ($O(T^2)$ in the horizon). **DAgger** fixes this by querying the expert on the
  learner's own states. When the expert is your planner, that is cheap.
* **Multimodality.** Demonstrations often contain several valid ways to do something
  (go left or right around an obstacle). MSE regression averages them into a collision.
  Mixture models, discretisation, **diffusion** and **flow-matching** policies represent
  multimodal action distributions. **Action chunking** (ACT) predicts short action
  sequences, which reduces compounding error and jitter. Diffusion Policy combines both
  ideas with receding-horizon execution.
* **RL.** PPO (on-policy, robust, loves parallel simulation) and SAC (off-policy, sample
  efficient). Thousands of parallel environments on a GPU (Isaac Lab, MuJoCo
  Playground, MJX) make on-policy RL practical. Reward design is most of the work.
  **Residual RL** learns a correction on top of a classical controller and is often the
  best of both.
* **Learning for planning.** Learned sampling distributions (e.g. CVAEs trained on
  successful plans) and learned IK seeds speed up classical planners *without giving up
  their guarantees*. Neural planners (MPNet, Motion Policy Networks) and diffusion models
  over trajectories (Motion Planning Diffusion) amortise planning into a forward pass.
  GPU-parallel classical methods (cuRobo) are the baseline to beat.
* **Foundation models.** Vision-language-action models (RT-2, OpenVLA, π0 and
  successors) map images and language to actions. They are strong on semantics and
  generalisation, and weak on precision and guarantees. They usually sit *above* a
  classical motion and control stack, or are fine-tuned per embodiment (LeRobot makes
  this accessible).
* **Safety pattern: learned proposal, classical verification.** Let the network propose
  motion chunks, check them with your collision checker and limits, and fall back to the
  planner when a chunk fails. This is how you deploy learned motion without betting the
  robot on it.
* **Evaluation.** Report success rates with confidence intervals, separate in- and
  out-of-distribution test sets, latency and compute, and a strong classical baseline
  with the same information. Ablate everything.

## Study materials

**Core**

| Resource | Take from it |
|---|---|
| S. Levine, UC Berkeley **CS 285: Deep Reinforcement Learning** ([course site](https://rail.eecs.berkeley.edu/deeprlcourse/), lectures on YouTube) | Imitation learning, policy gradients, actor-critic, model-based RL. Watch lectures 1–9 at least |
| R. Sutton & A. Barto, *Reinforcement Learning: An Introduction* (2nd ed., [free](http://incompleteideas.net/book/the-book-2nd.html)) | Foundations |
| [OpenAI Spinning Up](https://spinningup.openai.com/) | Clean explanations and implementations of PPO and SAC |
| C. Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion", RSS 2023 ([project page](https://diffusion-policy.cs.columbia.edu/)) | The modern BC baseline |
| T. Zhao, V. Kumar, S. Levine & C. Finn, "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware", RSS 2023 (ACT) | Action chunking with transformers |

**Deeper**

| Resource | Take from it |
|---|---|
| S. Ross, G. Gordon & J. A. Bagnell, "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning", AISTATS 2011 (DAgger) | Why BC compounds errors, and the fix |
| A. Mandlekar et al., "What Matters in Learning from Offline Human Demonstrations for Robot Manipulation", CoRL 2021 (robomimic) | Careful empirical methodology for BC |
| A. Fishman et al., "Motion Policy Networks", CoRL 2022 | Distilling a classical planner into a point-cloud policy, for a Franka. The closest paper to the mini project |
| B. Ichter, J. Harrison & M. Pavone, "Learning Sampling Distributions for Robot Motion Planning", ICRA 2018 | Learned samplers inside RRT/PRM |
| A. H. Qureshi et al., "Motion Planning Networks", IEEE T-RO 2021; J. Carvalho et al., "Motion Planning Diffusion", IROS 2023 | Neural and generative planners |
| N. Rudin et al., "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning", CoRL 2021 | Massively parallel on-policy RL |
| M. Kim et al., "OpenVLA" (2024); K. Black et al., "π0: A Vision-Language-Action Flow Model for General Robot Control" (2024) | Vision-language-action models |
| Tools: [LeRobot](https://github.com/huggingface/lerobot), [Isaac Lab](https://isaac-sim.github.io/IsaacLab/), [MuJoCo Playground](https://playground.mujoco.org/), [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3), [CleanRL](https://github.com/vwxyzjn/cleanrl) | What you'll train with |

## Warm-ups

1. **The averaging failure.** Generate 2-D demonstrations that pass left *or* right of an
   obstacle. Train an MSE MLP, a mixture density network and a small diffusion model.
   Plot their rollouts.
2. **Compounding error.** Train BC on your Module 16 planner's 2-D paths, measure success
   against horizon, then run DAgger with the planner as expert and plot the improvement.
3. **PPO from scratch** (CleanRL-style, about 300 lines) on a MuJoCo reacher. Then use a
   GPU-parallel environment and compare wall-clock time.
4. **Residual RL.** On top of your Module 12 impedance controller, learn a residual policy
   for peg-in-hole with 1 mm clearance in MuJoCo. Compare success against impedance alone.

## Mini project: planner distillation, a learned motion policy with a classical safety net *(B)*

**Data.** Use your Module 16/17 pipeline to generate ≥ 100,000 collision-free,
time-parameterised FR3 trajectories in randomised tabletop and shelf scenes (randomise
obstacle poses and sizes, start configurations and goal poses). Log the joint state, goal
pose, a scene encoding (start with obstacle parameters, then a point cloud from a
simulated depth camera) and the action chunks.

**Models**
1. MLP BC predicting the next $k$ joint deltas.
2. ACT-style transformer with action chunking.
3. Diffusion Policy (temporal U-Net or transformer) over action chunks.
4. *(Stretch)* A point-cloud policy in the style of Motion Policy Networks.

**Deployment.** A ROS 2 node running the policy at 10–20 Hz. Each predicted chunk is
validated by `mamp_collision` and joint-limit checks, and executed through your Module 12
controller (or short JTC goals) in MuJoCo via `mujoco_ros2_control`. Invalid chunks
trigger a fallback to RRT-Connect.

**Proof of competence**

* Results on **in-distribution** and **out-of-distribution** scenes (new obstacle
  shapes, unseen shelf heights), each with success rate ± 95% confidence interval, path
  length, smoothness, inference latency and the fraction of chunks rejected by the
  safety layer.
* An honest comparison with the expert planner: when does the policy win (latency,
  smoothness) and when does it lose (OOD generalisation, guarantees)?
* Ablations: chunk length, number of diffusion steps, and a dataset-size scaling curve.
* A failure gallery with a diagnosis for each class of failure.

**Stretch**

* Use the trained model as a **learned sampler** inside your RRT-Connect (bias samples
  towards predicted trajectories) and measure the speed-up while keeping completeness.
* Fine-tune an open VLA (via LeRobot) on a language-conditioned pick task in simulation
  ("pick the red cube"), with your classical stack executing the motions.

## Self-check

1. Why does behaviour cloning suffer from compounding errors, and how does DAgger fix it?
2. Why do diffusion and flow policies handle multimodal demonstrations better than MSE
   regression?
3. What does action chunking buy, and what does it cost?
4. For manipulation, when would you choose RL over planning plus control, and when the
   reverse?
5. Explain "learned proposal, classical verification". What guarantees survive?
6. How do you compare a learned policy fairly with a classical planner?
7. What can go wrong with domain randomisation?

Next: [Module 22: Capstone](22-capstone.md)
