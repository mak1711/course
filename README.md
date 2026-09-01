# Go2 Natural-Language Navigation

Say **"go to the sofa"** or **"what's in this room?"** in plain English, and a simulated
Unitree Go2 quadruped actually walks there — planning its own path, avoiding obstacles,
and (in the real-walking demo) looking around with its camera to find things that were
never hand-labeled anywhere.

```
 you: "what's in this room?"
   -> LLM (Gemini) calls rotate()/navigate_to_point()/list_detected_objects()
   -> camera + YOLO-World actually looks and reports back
 you: "go to the yellow cylinder"
   -> LLM calls navigate_to_place("yellow cylinder")
   -> Nav2 plans a path and the robot walks there
```

## What this is

A ROS 2 (Humble) project that connects a large language model to a quadruped robot's
navigation stack through the **Model Context Protocol (MCP)**: an LLM never touches
motors or coordinates directly — it calls typed tools (`navigate_to_place`,
`rotate`, `navigate_to_point`, `list_detected_objects`, ...), and Nav2 does the actual
path planning, obstacle avoidance, and (for the real-walking robot) balance control.
The LLM decides *what* to do and *when*; the robot stack decides *how*.

Two independent simulated robots run the same LLM/navigation stack, on purpose:

| | `go2_simulation` (`./run_go2_demo.sh`) | `unitree_go2_ros2_jazzy` (`./run_go2_demo_junior.sh`) |
|---|---|---|
| Legs | locked, slides as a rigid block | real revolute joints, actual walking gait |
| Purpose | test Nav2/LLM without gait-stability risk | the real thing — genuine balance control |
| World | one room, hand-labeled objects (sofa, table) | two rooms + doorway, camera-discovered objects |
| Object knowledge | fixed places in `places.yaml` | **none pre-baked** — finds things live via camera |

## How it works (architecture)

```
person types/speaks a request
        |
        v
go2_llm_nav  --------- talks to --------->  Gemini (or any OpenAI-compatible LLM)
   (agent.py: system prompt + tool loop)
        |
        v  (Model Context Protocol, stdio)
go2_mcp_server  <--- reads -----  go2_semantic_map (places.yaml: name -> approach pose)
   (server.py: MCP tools)
        |
        v  (ROS 2 actions/topics)
Nav2  (AMCL localization, costmaps, path planning, DWB/behavior-tree control)
        |
        v
Gazebo simulation  (physics, sensors: lidar, RGB-D camera)
        |
        +--> YOLO-World (yolo_ros) --> object detections --> back into go2_mcp_server
```

## Packages

- **`src/go2_llm_nav`** — the LLM front end (`agent.py`'s system prompt + tool-calling
  loop; `go2_llm_nav`/`go2_llm_nav_web`/`go2_llm_nav_gui` are terminal/browser/desktop
  entry points to the same agent).
- **`src/go2_mcp_server`** — the MCP server exposing navigation as LLM-callable tools
  (`navigate_to_place`, `rotate`, `navigate_to_point`, `get_map_overview`,
  `list_detected_objects`, `get_navigation_status`, `cancel_navigation`,
  `list_ros_topics`).
- **`src/go2_semantic_map`** — `places.yaml`: named place -> approach pose. Deliberately
  has **no object entries** for the junior/real-walking demo — see `USAGE.md` for why.
- **`src/go2_navigation`** — Nav2 + `slam_toolbox` configs, launch files, and saved maps
  for both demos.
- **`src/go2_simulation`** — the locked-leg sliding robot's URDF/world/Gazebo bridge.
- **`unitree_go2_ros2_jazzy1/`** — a second colcon workspace: the real-walking robot
  (`unitree_guide2`'s balance/gait controller), its world (`default.sdf`, now two rooms
  + a doorway), and a vendored `yolo_ros` (YOLO-World object detection) clone with
  project-specific patches.
- **`unitree_ros2/`** — the real-hardware DDS bridge, kept for eventually connecting to
  an actual physical Go2 (not used by either simulation demo).
- **`src/go2_real_bridge`** — bridges the real Go2's SDK topics (position/IMU,
  lidar) into what `slam_toolbox`/Nav2 already expect (`nav_msgs/Odometry`,
  `odom`→`base_link` TF, `/scan`) — `unitree_ros2` provides the raw SDK data but
  publishes no TF and no standard `Odometry` message at all. See `USAGE.md` for
  building a 2D map from the real robot.

## Get started

- **[INSTALL.md](INSTALL.md)** — one-time setup, step by step, from a clean machine.
- **[USAGE.md](USAGE.md)** — how to run the demos and what you can say to the robot.
- **[PROGRESS.md](PROGRESS.md)** — the full development history: every bug found and
  fixed, why each design decision was made. Not needed to use the project, but the
  place to look if something behaves unexpectedly and you want the "why."
