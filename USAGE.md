# Using Go2 Natural-Language Navigation

Assumes you've already done the one-time setup in [INSTALL.md](INSTALL.md).

## Two demos — which one to run

- **`./run_go2_demo.sh`** — `go2_simulation`, a simplified robot whose leg joints are
  locked in its URDF. It slides around the floor as a rigid block, kinematically
  following `/cmd_vel` — no walking, no balance to worry about. Good for testing the
  SLAM/Nav2/LLM pipeline in isolation. One room, three hand-labeled places: `sofa`,
  `table`, `entrance`.
- **`./run_go2_demo_junior.sh`** — the real Go2 controller (`junior_ctrl`/
  `unitree_guide2`): genuine revolute leg joints, a real balance/gait controller. It
  stands itself up and walks. Two rooms connected by a doorway, a camera, and
  YOLO-World object detection — **no objects are pre-labeled**; the robot has to
  actually look to find out what's in the room (see "What you can ask" below).

Both are driven by the identical LLM/MCP/Nav2 stack — only the robot and world differ.

## Quick start

```bash
cd /home/kan/lab/course
./run_go2_demo.sh          # sliding robot, hand-labeled room
# or
./run_go2_demo_junior.sh   # real walking robot, camera + object detection
```

Each script starts everything in order — Gazebo, Nav2 (localized against the saved
map), object detection (junior only), then confirms your Gemini API key — waiting for
each piece to actually be *ready*, not just launched — before opening a chat window.
Windows that appear:

- **Gazebo** — the 3D room/robot.
- **RViz** — the map, robot model, laser scan, and the AMCL particle cloud.
- **"Go2 Navigator"** — the chat window (a real desktop app by default): known places
  as clickable buttons, a live status badge, a chat box. Every reply shows the actual
  tool call(s) and their raw result above the answer — if a reply claims something
  happened with no matching `[tool call]`/`[tool result]` line above it, it didn't.

Closing one window just closes that view. To fully stop everything, go back to the
terminal you ran the script from and press **Ctrl-C**.

Prefer a browser tab or plain terminal chat? `GO2_UI=web ./run_go2_demo.sh` (opens
`http://127.0.0.1:8765`) or `GO2_UI=cli ./run_go2_demo.sh`.

## What you can ask

**Named places** (either demo): "go to the sofa", "go to the waypoint" — ask "what
places do you know?" to list them, "how's it going?" to check progress, "cancel"/"stop"
to abort. The robot will only go to a place it actually knows about; it won't invent a
destination.

**Finding things** (junior demo only — this is the point of that world having no
pre-baked object list): ask *"what's in this room?"* or *"look around"* or *"find the
yellow cylinder"*. The agent has three raw movement primitives and decides how to
combine them itself — there's no scripted "look around" routine:

- `get_map_overview()` — the known map (obstacles / free space / unexplored), no
  labels.
- `rotate(angle_deg)` — turn in place by an angle it picks, to let the camera see more
  from where it's standing.
- `navigate_to_point(x, y)` — drive anywhere on the map, for any reason (not limited to
  a named place).

Once something's been seen (`list_detected_objects()`), you can say *"go to the yellow
cylinder"* and it navigates straight there, no need to re-explore.

## Environment variables

| Variable | Default | Effect |
|---|---|---|
| `GEMINI_API_KEY` / `GO2_LLM_API_KEY` | *(required)* | Gemini API key. |
| `GO2_LLM_MODEL` | `gemini-3.1-flash-lite` | LLM model — this one has a generous free-tier quota and has been reliable for tool-calling. Newer Gemini models can have very tight daily limits. |
| `GO2_LLM_BASE_URL` | Gemini's OpenAI-compatible endpoint | Point this at any other OpenAI-compatible endpoint (e.g. a local Ollama server) to swap the LLM entirely. |
| `GO2_UI` | `gui` | `gui` (desktop window) / `web` (browser tab) / `cli` (terminal chat). |
| `GO2_DETECTION` | `on` | `off` skips YOLO-World entirely (faster startup, junior demo only). |
| `GO2_PLACES_FILE` | set by each script | Which `places.yaml` to load — lets you point at a custom one. |

## Manual / step-by-step version

Useful for understanding what's happening, or running pieces separately (e.g. leave
the simulation running and just restart the LLM).

```bash
# Terminal 1 -- simulation (junior/walking robot shown; swap for go2_simulation's
# sim.launch.py to run the sliding-robot demo instead)
source /opt/ros/humble/setup.bash
source /home/kan/lab/course/unitree_go2_ros2_jazzy1/install/setup.bash
ros2 launch unitree_go2_sim unitree_go2_nav_bringup.launch.py

# Terminal 2 -- Nav2, using the saved map
source /opt/ros/humble/setup.bash
source /home/kan/lab/course/install/setup.bash
ros2 launch go2_navigation nav2_junior.launch.py

# Terminal 3 -- object detection (skip for the sliding-robot demo, it has no camera)
source /opt/ros/humble/setup.bash
source /home/kan/lab/course/unitree_go2_ros2_jazzy1/install/setup.bash
ros2 launch yolo_bringup yolo-world.launch.py \
  input_image_topic:=/rgbd_camera/image input_depth_topic:=/rgbd_camera/depth_image \
  input_depth_info_topic:=/rgbd_camera/camera_info target_frame:=map threshold:=0.02 \
  use_3d:=True use_tracking:=False depth_image_units_divisor:=1 device:=cuda:0

# Terminal 4 -- the LLM chat interface (needs GEMINI_API_KEY set)
source /opt/ros/humble/setup.bash
source /home/kan/lab/course/install/setup.bash
export GO2_PLACES_FILE=/home/kan/lab/course/src/go2_semantic_map/config/places_junior.yaml
ros2 run go2_llm_nav go2_llm_nav_gui   # or go2_llm_nav_web / go2_llm_nav
```

Nav2 needs to know where the robot actually is when it starts — both launcher scripts
read the robot's live position from `/odom` and publish it as the initial pose
automatically (see `PROGRESS.md` for why this has to extract yaw properly from the full
quaternion rather than assume the robot is perfectly flat). Doing this manually:

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}' -r 5
```
(only correct if the robot is genuinely still at the origin — check `/odom` first
otherwise).

## Extending: adding a new place

Edit `src/go2_semantic_map/config/places.yaml` (sliding demo) or
`places_junior.yaml` (junior demo) — add a name, an x/y position + yaw, and a
description, then rebuild:

```bash
cd /home/kan/lab/course
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_semantic_map
```

The junior demo's `places_junior.yaml` deliberately has no object entries by design —
adding one there defeats the point of the live-detection feature (the agent will just
offer the known name instead of actually looking). It's the right file to edit for
generic navigation reference points (like the existing `waypoint`/`start`), not for
objects the camera can already find on its own.

## Known rough edges

- **Gemini free-tier rate limits (429).** Wait a bit and try again, or switch models
  with `GO2_LLM_MODEL`. The agent retries automatically on `429`/`5xx` a couple of
  times before giving up and telling you plainly, rather than crashing the chat.
- **Camera rendering is intermittently unreliable** on NVIDIA-hybrid-graphics laptops —
  frames can occasionally go blank mid-session even with the EGL workaround applied
  (see INSTALL.md). If object detection stops finding anything the robot has clearly
  driven past, this is the likely cause; a fresh restart of the sim usually clears it.
  Not yet root-caused at the driver level — see `PROGRESS.md`.
- **Turn speed is capped at 0.3 rad/s** on the real-walking robot (`rotate()`, and any
  Nav2 turning) — this is the actually-tested-safe ceiling for this machine's gait
  controller; going faster caused a real fall during testing. See `PROGRESS.md` for the
  full investigation if you're tuning this.
- **The junior demo's map only covers a ~12.5x12m two-room area.** Points outside that
  (via `navigate_to_point`) will fail or behave unpredictably — `get_map_overview()`
  only ever shows the explored region.
