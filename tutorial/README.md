# The Robot Arm Story

How do you get a robot arm to move its hand somewhere useful without hitting anything?
That one question is the whole of *Modelling and Motion Planning*. This tutorial answers
it in six chapters, one layer at a time. Each layer is a problem the previous one leaves
open.

```mermaid
flowchart LR
    A["1 · Talking<br/>How robot programs<br/>work together"] --> B["2 · Space<br/>Where is everything?"]
    B --> C["3 · Kinematics<br/>Joints ↔ hand"]
    C --> D["4 · Forces & time<br/>Making it actually move"]
    D --> E["5 · Planning<br/>Finding a way through clutter"]
    E --> F["6 · Real world<br/>Frameworks, simulation, learning"]
```

| Chapter | The question it answers |
|---|---|
| [1 · Talking](1-talking.md) | How do dozens of small programs cooperate to run one robot? (ROS 2) |
| [2 · Space](2-space.md) | How do we describe *where* and *which way round* things are? |
| [3 · Kinematics](3-kinematics.md) | Joint angles → hand pose, and the much harder reverse |
| [4 · Forces & time](4-forces-and-time.md) | What torques make that happen, and how fast may we go? |
| [5 · Planning](5-planning.md) | How do we get from A to B without hitting anything? |
| [6 · Real world](6-real-world.md) | How do the pros glue it together, test it, and teach it? |

## How each lesson works

```mermaid
flowchart LR
    S["📖 The story<br/>intuition + pictures"] --> L["🔎 Look up<br/>search terms + best sources"] --> P["🛠 Prove it<br/>a build with a pass mark"]
```

Lessons give you the *shape* of an idea, never the full detail. The **Look up** line
lists the exact terms to search and the best place to read about them. The **Prove it**
project is how you find out whether you really understood. A project counts as passed
only when a number says so.

## Two lanes

```mermaid
flowchart TB
    subgraph Fast["🚀 Fast lane: course essentials (~200 h)"]
        direction LR
        f1[1] --> f2[2] --> f3[3] --> f4["4 (control & timing)"] --> f5["5 (search & sampling)"] --> f6["6 (MoveIt)"]
    end
    subgraph Full["🏔 Full journey: everything + every project (~500 h)"]
        direction LR
        g1["Fast lane"] --> g2["Dynamics, collision,<br/>optimisation, constraints"] --> g3["Simulation,<br/>learning"] --> g4["Capstone"]
    end
    Fast --> Full
```

```mermaid
timeline
    title Suggested pace at ~12 h per week
    Weeks 1-2 : Chapter 1 : ROS 2 fluency
    Weeks 3-4 : Chapter 2 : Do Lab 1 here
    Weeks 5-7 : Chapter 3 : Do Lab 2 here
    Weeks 8-9 : Chapter 4 : Control and timing
    Weeks 10-12 : Chapter 5 : Search and sampling
    Weeks 13-22 : Full journey : Dynamics, collision, optimisation
    Weeks 23-36 : Chapter 6 : Simulation, learning, capstone
```

## Where the labs fit

```mermaid
flowchart LR
    L1a["Lab 1 · build & launch"] --> C1["Ch 1"]
    L1b["Lab 1 · TF tree, /tf"] --> C2["Ch 2"]
    L1c["Lab 1 · URDF links"] --> C2
    L1d["Lab 1 · DH forward kinematics<br/>+ rotation conversions"] --> C3["Ch 3"]
    L1d --> C2
    L2a["Lab 2 · ros2_control bring-up"] --> C4["Ch 4"]
    L2b["Lab 2 · waypoints → IK → trajectory"] --> C3
    L2b --> C4
    L2c["Lab 2 · plasma-cutter essay"] --> C5["Ch 5"]
```

## Ground rules

* **Integrity.** The labs are assessed. This tutorial explains the ideas behind every
  TODO but never gives the answers. Build projects in your own workspace
  (`~/mamp_ws`), and check course policy before publishing anything that overlaps a
  lab.
* **Versions.** Do coursework on ROS 2 **Humble** (Ubuntu 22.04). Jazzy and Lyrical
  are the newer long-term releases; use them for side projects if a tool needs them.
* **Method.** Sketch the maths on paper → code it yourself → compare against a trusted
  library → measure. If you can't explain it to a friend in two minutes, go round again.

## The bookshelf

Four sources cover most of this tutorial. Everything else is linked from the lesson
that needs it.

* **Lynch & Park, *Modern Robotics*** (free, with short videos):
  [hades.mech.northwestern.edu](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
* **Siciliano et al., *Robotics: Modelling, Planning and Control*** (Springer, 2009)
* **LaValle, *Planning Algorithms*** (free): [lavalle.pl/planning](https://lavalle.pl/planning/)
* **ROS 2 docs** (Humble): [docs.ros.org/en/humble](https://docs.ros.org/en/humble/)

Start here → [Chapter 1 · Talking](1-talking.md)
