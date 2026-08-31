# Go2 Autonomous Natural-Language Navigation — Project Progress

**Last updated:** 2026-08-31 (Session 13)

**End goal:** `"Go to the sofa."` → LLM/agent → semantic place lookup → Nav2 goal → autonomous
navigation → Unitree Go2. Built and validated in simulation first, real robot later.

**Want to just run it?** See `USAGE.md` — one command (`./run_go2_demo.sh`) brings up
the whole stack and drops you into a chat prompt. This file (`PROGRESS.md`) is the full
technical history/architecture; `USAGE.md` is deliberately just "how do I use it," and
`INSTALL.md` covers one-time setup. `README.md` is the project summary/architecture
overview.

Pipeline: Go2 ROS 2 interface → 3D LiDAR → `pointcloud_to_laserscan` → TF/odometry →
`slam_toolbox` (mapping) → saved map → AMCL (localization) → Nav2 (planning + obstacle
avoidance) → semantic map (`sofa → PoseStamped`) → LLM/MCP tools (`list_places`,
`navigate_to_pose`, `get_navigation_status`, `cancel_navigation`). The LLM never touches
`/cmd_vel` directly or invents coordinates — Nav2 stays in charge of navigation.

---

## Current status at a glance

| Milestone | Status |
|---|---|
| 1. Go2 sim: motion, odom, TF, 3D LiDAR → `/scan` | ✅ Done, verified live |
| 2. slam_toolbox mapping + map saving | ✅ Done, verified live |
| 3. Nav2 bringup, AMCL, goal execution | 🟡 Working — goal-arrival precision tuning deprioritized (not a current priority) |
| 4. Robust planar-base physics (no tipping) | ❌ Deferred — real fix identified (planar virtual-joint chain / a proper controller e.g. `champ`), not a current priority |
| 5. Semantic map (`go2_semantic_map`) | ✅ Done — manual YAML, name → approach pose |
| 6. MCP layer (`go2_mcp_server`) | ✅ Done — 4 tools, verified over real MCP stdio protocol |
| 7. LLM layer (`go2_llm_nav`) | ✅ Done — free local LLM (Ollama), full "go to the sofa" → real robot motion pipeline verified live |
| 8. Real quadruped walking (`unitree_go2_ros2_jazzy`) | 🟡 Working, not yet stable — see below |
| 9. Wire real walking into Nav2/SLAM/LLM | ✅ Wired; dramatically more stable — root cause was a broken GPU driver (RTF ~0.76→~0.997 after reboot), not the controller. One fall still occurred during extended testing — much rarer, not zero-risk |

---

## Workspace layout

```
/home/kan/lab/course/
├── unitree_ros2/         # existing, untouched — real-robot DDS bridge (msgs + example clients)
├── src/
│   ├── go2_simulation/    # Gazebo (gz-sim/Ignition Fortress) sim: URDF, world, sensors, bridge
│   ├── go2_navigation/    # slam_toolbox + Nav2 config/launch, saved maps
│   ├── go2_semantic_map/  # manual name -> approach-pose lookup (places.yaml)
│   ├── go2_mcp_server/    # MCP server: list_places / navigate_to_place / get_navigation_status / cancel_navigation
│   └── go2_llm_nav/       # LLM front end: text prompt -> MCP tool calls (via local Ollama)
├── build/, install/, log/ # this workspace's colcon output (separate from unitree_ros2's own)
└── PROGRESS.md            # this file
```

`unitree_ros2` was deliberately left as its own standalone workspace rather than moved under
`src/`, to avoid touching anything already working there. `go2_simulation`/`go2_navigation`
build and run independently of it (plain `rmw`, not `rmw_cyclonedds_cpp` — that override is
only needed to reach the real robot's DDS network).

Also present on this machine (not part of this workspace, referenced only as background
context): `~/ros2_ws/src/go2_robot_sdk` — a separate, real-robot WebRTC driver project. Its
`package.xml` self-describes as an **"unofficial sdk for Unitree Go2"** (community project,
not from Unitree) — its `go2.urdf` and meshes were the source for `go2_simulation`'s robot
description. See "URDF provenance" under Milestone 1 for exactly what was and wasn't
changed.

---

## Milestone 1 — Simulation core (DONE)

**Package:** `go2_simulation`

### URDF provenance (what's official vs. what we changed)

The URDF was **not** downloaded from Unitree's own GitHub — it was copied from
`~/ros2_ws/src/go2_robot_sdk`, a third-party/community project that explicitly labels
itself `"unofficial sdk for Unitree Go2"`. That said, the file's own header comment
(`"This URDF was automatically created by SolidWorks to URDF Exporter"`) is the standard
boilerplate found on Unitree's real CAD-exported description, and the link/joint naming
(`FL_hip`, `FL_thigh`, `FL_calf`, …), masses, inertias, joint origins, and mesh files
(`base.dae`, `hip.dae`, `thigh.dae`, …) match the description that circulates across the
whole Go2 ecosystem. **We did not invent or redesign any robot geometry** — no link was
resized, repositioned, or given different mass/inertia than the source file.

What we actually changed, explicitly:
- **Modified:** all 12 leg joints (`FL_hip_joint`, `FL_thigh_joint`, `FL_calf_joint`, ×4
  legs) changed from `revolute` → `fixed`, locked at their zero pose. This is a real
  change to the *kinematic configuration*, not cosmetic — the real robot's legs are
  actuated; ours are rigid. Done because Milestone 1 only needs planar 2D motion for Nav2
  testing, not a walking gait. If a later milestone needs actuated legs (e.g. a real gait
  controller), this needs revisiting.
- **Removed:** two links/joints (`map`, `odom`) that the *unofficial* source project had
  added as fake static frames for its own visualization — not part of Unitree's robot,
  not touched from the "official" geometry.
- **Added** (pure additions, nothing official removed or altered): a new `lidar_link`
  frame + Gazebo `gpu_lidar` sensor tag, an `<sensor type="imu">` tag on the existing
  `imu` link, and Gazebo-only simulation plugins (`VelocityControl`, `OdometryPublisher`).
- **Cosmetic only:** `package://go2_robot_sdk/...` mesh paths rewritten to
  `package://go2_simulation/...` (same mesh files, just re-hosted in our package).

- All 12 leg joints locked `fixed` (see above; no gait simulated — out of scope, Nav2 only
  needs planar motion) and gravity disabled on the resulting lumped `base_link` rigid body.
- Planar motion driven kinematically via gz-sim's `VelocityControl` system plugin
  (`/cmd_vel` → `/model/go2/cmd_vel` via `ros_gz_bridge`).
- Real, physics-tracked odometry via gz-sim's `OdometryPublisher` plugin → `/odom` and the
  dynamic `odom → base_link` TF (not faked).
- IMU sensor → `/imu` (~100 Hz).
- Simulated 3D LiDAR (`gpu_lidar`, 360° horizontal × 16 vertical channels, 20 m range) on a
  new `lidar_link` (clean orientation, unlike the reference project's oddly-rotated `radar`
  link, which is kept only for visual parity) → PointCloud2 → `pointcloud_to_laserscan` →
  `/scan` (~10 Hz).
- `worlds/go2_world.sdf`: 10×10 m room, 4 walls, 2 pillars, 2 box "furniture" obstacles
  (stand-ins for later semantic places, e.g. a "sofa").
- Runs headless (`gz sim -s`) with forced Mesa software rendering
  (`LIBGL_ALWAYS_SOFTWARE=1`, `MESA_GL_VERSION_OVERRIDE=3.3`) — this machine's NVIDIA
  driver has a kernel/userspace version mismatch, no working GPU-accelerated EGL context.

**Verified live:** `/cmd_vel` → real odometry motion; TF `odom→base_link` (dynamic) →
`base_link→lidar_link`/`imu`/legs (static); `/scan` at 10 Hz with real obstacle ranges;
`/imu` at 100 Hz.

**Bug found and fixed:** the LiDAR was initially self-hitting the robot's own fixed legs
(min range 0.15 m was inside leg geometry), poisoning the map with noise right at the
robot's start position and blocking Nav2's planner. Fixed by raising `range/min` to 0.4 m
and narrowing the downward vertical FOV.

---

## Milestone 2 — SLAM (DONE)

**Package:** `go2_navigation` (`config/mapper_params_online_async.yaml`,
`launch/slam.launch.py`)

- `slam_toolbox` online-async mapping, `base_frame: base_footprint`, `min_laser_range: 0.4`
  (matches the LiDAR fix above).
- Full-room exploration (rotate-in-place + bounded visits to all 4 quadrants, kept well
  clear of walls) produces a clean, correctly-proportioned map: full room boundary, both
  pillars resolved as circles, both box obstacles' corners visible.
- Map saved to `go2_navigation/maps/go2_world_map.{pgm,yaml}` via `nav2_map_server
  map_saver_cli`.

**Lesson learned:** early exploration attempts that drove fast/close to walls produced a
corrupted, wildly-oversized map (robot clipping through walls — see Milestone 4 below for
why) or, later, a map corrupted by the robot silently tipping over mid-drive. A careful,
margin-respecting exploration pattern with position/orientation checks between legs is
what actually produced a clean map.

---

## Milestone 3 — Nav2 (WORKING, not fully tuned)

**Package:** `go2_navigation` (`config/nav2_params.yaml`, `launch/nav2.launch.py`)

- Nav2 bringup (AMCL + planner_server + controller_server(DWB) + bt_navigator + costmaps),
  params adapted for Go2's footprint (`robot_radius: 0.35`).
- AMCL tuning history:
  - Defaults (`alpha 0.2`, `update_min_d/a 0.25/0.2`): first goal test "succeeded" but the
    robot's true final position was 0.69 m from the target — the local controller's goal
    check runs in the `odom` frame, computed once at planning time, and AMCL's infrequent,
    large `map→odom` corrections let it go stale mid-transit.
  - Over-corrected to `alpha 0.05`, `update_min_d/a 0.1/0.05` (very frequent small
    corrections): made it *worse* — 19 recovery behaviors, repeated "Failed to make
    progress", final position ~2 m off. Hypothesis: corrections now happen so often that
    the local costmap's re-transformed static-layer obstacles jitter, confusing the local
    planner (unconfirmed, not proven).
  - Settled on a middle ground (`alpha 0.1`, `update_min_d/a 0.15/0.15`): goal reported
    `SUCCEEDED` with 5 recoveries, final live AMCL position 0.46 m from a (2,2) target —
    better than both extremes, but still outside the nominal 0.25 m `xy_goal_tolerance`.
- **Current read:** `NavigateToPose` reliably drives the robot across the room and around
  obstacles (real obstacle-avoidance behavior confirmed, not just luck — it explicitly
  re-plans around the pillar), and reports `SUCCEEDED`. The final-position precision
  (~0.46 m off a tight 0.25 m tolerance) is a known gap, but per direction this is **not a
  current priority** — good enough to build on for now. Revisit later if/when precise
  goal arrival actually matters (e.g. docking), most likely alongside the Milestone 4
  physics fix rather than as standalone controller tuning.

---

## Milestone 4 — Physical robustness (DEFERRED — root cause identified, not a current priority)

Per direction, not being worked on further right now — the sim is stable enough for
Milestones 5/6 as long as it isn't driven hard into obstacles. Notes kept below for when
this is picked back up. One concrete option raised: a proper quadruped controller (e.g.
`champ`) instead of the current kinematic hack — worth evaluating alongside the
planar-joint fix when this becomes a priority again.

While stress-testing Nav2, the robot was caught **silently tipping over** (roll/pitch
collapsing toward ~90°) after obstacle contact — this, not just AMCL tuning, was a real
contributor to the erratic navigation above (a tilted LiDAR feeds corrupted, non-planar
scans into SLAM/AMCL/costmaps).

Root cause: Milestone 1's config disables gravity on `base_link` and drives it purely
kinematically (`VelocityControl` overrides linear/angular velocity every physics step).
That plugin only manages the commanded axes (x, y, yaw) — any roll/pitch angular velocity
imparted by a contact event is never corrected, so a single bump leaves the robot
permanently tilted.

**Attempted fix:** re-enable gravity so the robot rests on its (still-rigid, fixed) legs
with real ground contact/friction, hoping natural contact stabilization would provide a
righting effect.
- It survived a **deliberate hard ram** into a pillar (roll/pitch stayed ~0.002 rad) —
  promising.
- But it **tipped anyway during ordinary driving+turning with zero obstacle contact** — the
  fixed, asymmetric leg/foot collision primitives dragging on real ground friction during a
  turn generate enough uneven torque to flip it on their own. This is a *worse* failure
  mode than gravity-off (which at least only fails on hard contact).
- **Reverted** to the gravity-off configuration (empirically the more stable of the two).

**Correct fix (not yet implemented):** replace the free-floating (gravity-off, kinematic)
`base_link` with a proper planar virtual-joint chain — `world → prismatic(local X) →
prismatic(local Y) → continuous(yaw) → base_link` — so roll, pitch, and Z motion are
*physically impossible* regardless of contact or friction, rather than merely
uncorrected-but-hopefully-rare. This is the single highest-priority piece of remaining
engineering work; it should also incidentally improve Nav2 goal precision (Milestone 3).

---

## Milestone 5 — Semantic map (DONE)

**Package:** `go2_semantic_map` (`go2_semantic_map/places.py`, `config/places.yaml`)

Manual, hand-authored lookup — no automatic object detection/labeling, per direction.
`places.yaml` maps a name to an **approach pose** (x, y, yaw in the `map` frame — not a
pose on top of the object, one that's actually reachable/free), with a human-readable
description. `load_places()` returns `{name: Place(x, y, yaw, description)}`;
`yaw_to_quaternion()` converts for `PoseStamped`. Currently defines 3 places matching the
test world: `sofa` (near `sofa_box`), `table` (near `table_box`), `entrance` (room center).

Adding a new place = adding one entry to `places.yaml`. No code changes, no rebuild needed
(it's read from the installed share directory at runtime).

---

## Milestone 6 — MCP layer (DONE)

**Package:** `go2_mcp_server` (`go2_mcp_server/server.py`, `go2_mcp_server/nav2_client.py`)

An MCP server exposing exactly the 4 tools from the original design, over the standard
MCP **stdio transport** (so any MCP client — Claude Desktop, an agent framework, etc. —
can be pointed at the installed executable and get an LLM-callable robot):

- `list_places()` — names + descriptions from the semantic map.
- `navigate_to_place(name)` — looks up the pose, sends a Nav2 `NavigateToPose` goal,
  returns as soon as it's **accepted** (does not block until arrival).
- `get_navigation_status()` — `idle` / `navigating` (+ distance remaining, elapsed time,
  recovery count) / `succeeded` / `aborted` / `canceled`.
- `cancel_navigation()` — cancels the active goal.

The LLM never touches `/cmd_vel` or invents coordinates — `navigate_to_place` only accepts
a name that exists in the semantic map; Nav2 does all the actual planning/driving.

**Implementation notes:**
- Uses the official `mcp` Python SDK (installed via `pip3 install mcp`, not an apt/rosdep
  package — **not yet added to any `package.xml`**, needs documenting/pinning properly if
  this is going to be relied on going forward). Note this SDK's API has moved on from the
  older `FastMCP`-style docs you'll find online: it's now `from mcp.server import
  MCPServer`, same `@mcp.tool()` decorator pattern otherwise.
- `nav2_client.py` wraps the `NavigateToPose` action client in a plain rclpy `Node` with
  **no background spin thread** — each call takes a lock and spins the node only as long
  as it needs (e.g. `spin_until_future_complete` to confirm goal acceptance,
  `spin_once(timeout=0.2)` to pump feedback when polled for status). This avoids two
  threads fighting over the same node/executor, at the cost of feedback only updating
  when something actually calls `get_navigation_status()` — fine for LLM-paced polling,
  would need revisiting for a tighter control loop.
- Tool handler functions are plain **sync** functions — the `mcp` SDK runs them off the
  event loop thread itself, so no async/await needed in our code.

**Verified end-to-end**, twice: first calling the tool logic directly against the live
sim+Nav2 stack (`sofa` — already-close instant success; `table` — real ~5 m cross-room
trip, watched `distance_remaining` count down), then again through an actual
`mcp.ClientSession` talking to the installed server over real stdio JSON-RPC (tool
schemas, `list_places` call, `get_navigation_status` call all correct). Also verified
`cancel_navigation` actually stops the robot (checked `/odom` twist == 0 after cancelling
mid-transit, not just that the call returned `True`).

---

## Milestone 7 — LLM layer (DONE)

**Package:** `go2_llm_nav` (`go2_llm_nav/agent.py`)

A free-text prompt (*"go to the sofa"*) → an LLM that can call the Milestone 6 tools →
real Nav2 motion. Per direction, used a **free** LLM so this could be tested without any
API key/account/cost: **Ollama**, running fully locally.

### Why Ollama, and how it was installed
This machine has no working GPU (same NVIDIA driver/kernel mismatch noted in Milestone 1)
and no passwordless `sudo`, so:
- The official `curl | sh` installer failed (needs `sudo`). Worked around with a **manual,
  user-local install**: downloaded the `ollama-linux-amd64.tar.zst` release asset
  directly from GitHub and extracted it to `~/.local/ollama` (no root needed). Note the
  installer script's own advertised tarball URL (`ollama.com/download/...tgz`) 404'd — the
  real current asset is `.tar.zst` from the GitHub releases page.
- Ran `ollama serve` from that local install (`OLLAMA_MODELS=~/.local/ollama/models`),
  confirmed it correctly falls back to CPU inference (`~10.5 GiB` available, integrated
  GPU present but dropped since `OLLAMA_IGPU_ENABLE` isn't set).
- Model: tried `llama3.1:8b` first (Meta's docs specifically call out strong native tool
  support), but its pull (4.9 GB) failed partway through on a transient DNS timeout to
  Ollama's storage backend, and even on retry was ~25 min at this network's ~3.3 MB/s.
  Switched to **`llama3.2:3b`** (2 GB, also supports tool calling, ~6 min to pull) to keep
  the test loop fast — good enough to prove the framework; a bigger/smarter model can be
  swapped in later via `GO2_LLM_MODEL` with no code changes if reasoning quality needs to
  improve.

### Architecture
`go2_llm_nav` is itself an MCP **client**: on each run it spawns `go2_mcp_server` as a
subprocess over stdio (same as the Milestone 6 test client), lists its tools, converts
their schemas to Ollama's (OpenAI-compatible) tool-calling format, and runs a standard
agentic loop against `POST /api/chat` on `http://127.0.0.1:11434` (via `httpx`): send
messages+tools -> if the model responds with tool calls, execute them through the MCP
session and feed the results back as `role: "tool"` messages -> repeat (capped at 8
rounds) until the model responds with plain text. Two run modes: a single prompt as a CLI
arg (`ros2 run go2_llm_nav go2_llm_nav "go to the sofa"`), or an interactive REPL when run
with no args.

Setup/runtime deps not yet captured in any `package.xml` (need doing properly later):
`pip3 install mcp httpx` (the `mcp` SDK pulls in `httpx2`, a differently-named fork — the
code needs plain `httpx`, installed separately), plus a running `ollama serve` and a
pulled model.

### Bugs hit and fixed
- `mcp`'s `Tool` object exposes `input_schema` (snake_case) in this SDK version, not
  `inputSchema` — fixed the schema-conversion code.
- `httpx` wasn't actually installed (only `httpx2`, a transitive dep of `mcp` under a
  different import name) — installed it explicitly.
- **The interesting one:** first live run of *"go to the sofa"* worked correctly right up
  until the model, after seeing `get_navigation_status` report `"navigating"` (not yet
  arrived), decided **on its own** to call `cancel_navigation` and told the person "the
  robot has stopped" — technically true, but not what was asked, and not something any
  tool told it to do. This is a **model-judgement problem, not a framework bug**: every
  layer below the LLM's decision-making (tool discovery, argument construction, MCP
  transport, Nav2 execution, the robot physically moving) worked correctly. Fixed by
  tightening the system prompt: don't poll/cancel a fresh goal just because it hasn't
  finished yet; only check status or cancel if the person actually asks. Confirmed the
  robot was left safely stopped (`/odom` twist `== 0`) either way — a small model
  second-guessing itself did not produce unsafe behavior, just an unhelpful one.

### Verified live, end to end
- `"go to the sofa"` → model called `navigate_to_place({'name': 'sofa'})` (no place names
  or coordinates invented) → real Nav2 goal, reported "Navigation has started," did not
  over-poll.
- Multi-turn (interactive REPL): `"go to the table"` → started a real navigation; follow-up
  `"how is the navigation going?"` → model correctly called `get_navigation_status` and
  reported `"still in progress"` **without** cancelling — confirmed via AMCL that the robot
  actually made a genuine ~3.5 m cross-room trip (from near the sofa to within ~0.4 m of
  the table's approach pose), stayed upright throughout.
- **Known limitation:** each CLI invocation of `go2_llm_nav` spawns a fresh
  `go2_mcp_server` subprocess, so `Go2NavClient`'s in-memory goal-tracking state does not
  persist *across separate invocations* — a status check needs to happen in the same
  interactive session (or process) that started the goal. Fine for this test; would need
  a persistent server (e.g. SSE/HTTP MCP transport instead of spawn-per-connection stdio)
  for a "real" multi-session deployment.

This is the originally-scoped end state: **a person's natural-language prompt drives the
real robot through Nav2, with the LLM restricted to naming a known place — never touching
`/cmd_vel`, never inventing a coordinate.** Confirmed working with a free, local, no-API-
key model, which was the point of this round of testing.

---

## Milestone 8 — Real quadruped walking (WORKING, not yet stable)

**Package:** `unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy` (user-provided, at
`/home/kan/lab/course/unitree_go2_ros2_jazzy1/`) — a **separate, self-contained** workspace,
not integrated with `go2_simulation`/`go2_navigation`/the semantic map/MCP/LLM layers above.
It has its own URDF, world, and controllers; nothing here talks to Nav2 yet.

### What this is
Directly answers the "is the Go2 actually walking, or just sliding with locked joints"
question from earlier: Milestones 1–7 above use `go2_simulation`'s deliberate simplification
(legs locked `fixed`, body translated directly via a velocity-control plugin) to get Nav2
working quickly. This package is the real thing — actuated legs, a genuine gait.

Five sub-packages, credited to their real authors (**not written by us**, only built/fixed):
- `unitree_go2_description` — Go2 URDF/xacro/meshes/worlds, `gz_ros2_control`-based (same
  modern gz-sim stack `go2_simulation` uses, not classic Gazebo, despite the parent repo's
  README claiming Ubuntu 24/Jazzy-only — see below).
- `unitree_go2_sim` — Gazebo launch/config.
- `ros2_unitree_legged_msgs` — `LowCmd`/`LowState`-style messages matching Unitree's real
  low-level SDK wire format.
- `ros2_unitree_legged_controller` — a custom `ros2_control` `ControllerInterface` plugin
  (`UnitreeLeggedController`, one instance per joint, effort command interface) bridging
  those messages to `ros2_control`.
- `unitree_guide2` — the actual high-level controller: a C++ FSM (Passive / FixedStand /
  Trotting / …) with QP-based (`quadProgpp`) whole-body balance control. Ported from
  Unitree's own reference `unitree_guide` project. Builds to an executable literally named
  **`junior_ctrl`** — that's what "junior control" referred to.

Two LiDARs are defined simultaneously in the URDF (Velodyne VLP-16 and a 4D "L1" lidar,
both as real `gpu_lidar` gz-sim sensor tags, both always bridged) — "exchangeable" in the
sense that you pick which topic to consume downstream (`/velodyne_points/points` vs.
`/unitree_lidar/points`), not a runtime toggle. Nothing needed changing there.

### Getting it to build and run on this machine (it wasn't, out of the box)
The repo's README claims Ubuntu 24.04/ROS 2 Jazzy/Gazebo Harmonic only. In practice, since
it already uses the version-abstracted `ros_gz_sim`/`gz_ros2_control` packages (not
classic Gazebo), it needed no porting — same `libignition-gazebo6`/Fortress stack
`go2_simulation` already runs on. What actually blocked it:

1. **A pre-existing package conflict on this machine**: `gazebo-plugin-base` (leftover
   classic Gazebo 11, unused by anything in this project) requires the `gazebo` metapackage,
   which hard-`Conflicts:` with `gz-tools2` (the modern suite everything else here needs).
   Fixed by removing the unused classic-Gazebo packages
   (`gazebo-common gazebo-plugin-base libgazebo-dev libgazebo11`).
2. `ros-humble-velodyne-gazebo-plugins` (initially in the install list, by analogy with the
   apt package names in the READMEs researched earlier) turned out to itself depend on
   classic `libgazebo11`/`ros-humble-gazebo-ros`, re-triggering the same conflict, and isn't
   actually needed — the URDF's Velodyne sensor is a plain gz-sim `<sensor type="gpu_lidar">`
   tag, not the classic plugin. Dropped from the install list.
3. Other plain apt deps needed (none of these are Gazebo-related, just genuinely missing):
   `ros-humble-gz-ros2-control`, `ros-humble-ros2-control(lers)`, `ros-humble-controller-manager`,
   `ros-humble-joint-trajectory-controller`, `ros-humble-joint-state-broadcaster`,
   `ros-humble-hardware-interface`, `ros-humble-controller-interface`,
   `ros-humble-generate-parameter-library`, `ros-humble-backward-ros`, `ros-humble-velodyne`,
   `ros-humble-velodyne-description`, `ros-humble-robot-localization`, `ros-humble-xacro`,
   and the plain (non-ROS) `liblcm-dev` (the LCM pub-sub library `junior_ctrl` links against).
   All required `sudo`, which this session doesn't have passwordlessly — the user ran these
   in their own terminal.
4. Built cleanly with plain `colcon build --base-paths unitree_go2_ros2_jazzy` (not
   `--symlink-install` — separately confirmed as broken for `ament_python` packages in this
   environment, see the note under "Useful commands"; this package is `ament_cmake` only,
   so it wasn't affected, just built plainly for consistency) from a workspace root at
   `unitree_go2_ros2_jazzy1/` (siblings `build/`, `install/`, `log/`, matching this project's
   convention). Only warnings (mostly the vendored `quadProgpp` QP solver using
   pre-C++17 `register` keywords), no errors.
5. **The real bug, and the one worth documenting carefully**: after building, the sim
   launched and spawned the robot fine, but every `ros2_control` controller spawner hung
   forever on `waiting for service /controller_manager/list_controllers` — because
   `controller_manager` never started at all. **No error was printed anywhere, including at
   `gz sim -v 4`.** Root cause: `ros-humble-gz-ros2-control` ships no ament environment hook
   that adds its plugin's install directory to gz-sim's plugin search path, so the
   `<plugin filename="gz_ros2_control-system" .../>` declared in the URDF silently fails to
   load — gz-sim just doesn't have `/opt/ros/humble/lib` on `GZ_SIM_SYSTEM_PLUGIN_PATH` and
   says nothing about it. Confirmed by exporting it manually
   (`GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH`) — the
   `controller_manager` service appeared immediately. **Fixed properly** (not just
   worked around) by adding a `SetEnvironmentVariable` action for it directly in both
   `unitree_go2_sim/launch/unitree_go2_launch.py` and `unitree_go2_launch_TI.py`, next to
   the existing `GZ_PARTITION` one — confirmed with a from-scratch launch using neither
   manual env var that controllers now come up on their own.
6. Separately hit, and worked around the same way as in `go2_simulation`: this machine's
   broken NVIDIA driver means headless rendering needs
   `LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3` set before launching (not baked
   into this package's launch files — same as our own).
7. **Resource contention, not a package bug**: several earlier debugging attempts were
   killed by only `kill`ing the top-level `ign gazebo` process, orphaning its
   `robot_state_publisher`/`parameter_bridge`/spawner children; combined with the old,
   unrelated `go2_simulation` + Nav2 stack from earlier sessions still running in the
   background, the machine hit a load average of 24 and `controller_manager`'s service
   calls started hanging purely from CPU starvation (not the plugin-path bug, which was
   already fixed by that point). Killed everything, confirmed idle CPU, retested clean.

### Verified live
- All 13 controllers (`joint_state_broadcaster` + 12× `UnitreeLeggedController`, one per
  leg joint) come up `active` on a clean launch, no manual intervention.
- `/joint_states` publishing real physics feedback at ~890 Hz.
- Ran `junior_ctrl` (needs a real TTY for its keyboard interface — no ROS topic control
  exists for it; used `screen` to give it a pty and `screen -X stuff` to inject keys):
  commanded `FixedStand` (key `2`) → watched the FSM's own tilt-safety telemetry go from
  spawn to `percent=1.000, tilt=1.000` (perfectly upright) over ~14s, final joint angles a
  plausible bent-knee stance (~0.67 rad thigh, ~-1.32 rad calf) — a real stand, not a
  teleport. Commanded `Trotting` (key `4`) then forward velocity (key `w`) → joint angles
  became asymmetric across legs and started cycling (genuine gait, not a static pose), and
  `/odom` moved **>1 m** in a few seconds — confirmed via real position readout, not
  inferred.
- **Not yet stable**: during the same trot, it fell over (the FSM's own safety monitor
  caught it: `tilt (rotMat(2,2)) = -0.615 < 0.5 (>60deg from vertical) -- forcing passive`
  — a real, working safety feature doing its job, not a crash). A first attempt on the
  bundled `elevation.sdf` (sloped) world fell immediately on spawn before `FixedStand` could
  even be commanded; switching to the flat `default.sdf` world fixed that specific failure,
  but the *walking* balance itself isn't tuned/stable yet in this environment.

### Honest bottom line (superseded by Milestone 9 below)
The full real-locomotion chain works end to end — URDF → `ros2_control` →
`gz_ros2_control` → real joint torques → `junior_ctrl`'s FSM/balance controller → genuine
standing and walking, physically verified via odometry, not assumed. At the time this was
written it wasn't wired to Nav2/SLAM/the LLM layer yet -- Session 7 (Milestone 9) did that.

---

## Milestone 9 — Wiring real walking into Nav2/SLAM/LLM (WORKING — instability RESOLVED, see update below)

**Update (Session 8, after a reboot):** the "known limit" this milestone originally ended
on turned out to have a real, fixable root cause, not a controller-tuning problem. See the
new subsection at the end of this milestone. Short version: it was a broken GPU driver
forcing CPU software rendering, which meant the simulation ran at RTF ~0.76 (76% of
real-time) while `junior_ctrl`'s balance loop ran at a fixed 500Hz on wall-clock time —
that mismatch, not the balance controller itself, was almost certainly what was causing
the repeated falls. After fixing the driver (a reboot), RTF measured ~0.997 and every
walking test that previously fell now completes cleanly, repeatably. Original Session 7
writeup preserved below unedited for the record; read to the end for the current status.

**Direction:** connect the Milestone 8 real-walking robot to the same Nav2/SLAM/LLM stack
Milestones 2–7 already validated on the sliding robot. Result: **it works** -- one full
autonomous `NavigateToPose` goal completed successfully with the robot staying upright the
entire time, via the real MCP tool logic, same as the sliding robot. The dominant limiting
factor turned out to be Milestone 8's already-documented walking-balance instability, not
the wiring itself -- see "What actually limits this" below.

### The Nav2-ready state transition already existed
`junior_ctrl`'s FSM has a state, `State_move_base` (`unitree_guide2/src/FSM/State_move_base.cpp`,
compiled in via the `COMPILE_WITH_ROS2_MB` flag CMakeLists already sets), that's a drop-in
subclass of `State_Trotting` except it takes velocity from a `/cmd_vel` subscription instead
of the keyboard -- exactly Nav2's `controller_server` output. Entered via keyboard key `5`
from `FixedStand`. This wasn't something to build, just something to find and trigger.

### Automating the keyboard-only startup (`auto_stand.py`)
`junior_ctrl` has no ROS topic/service for FSM state control at all -- only a raw-TTY
keyboard reader (confirmed in Milestone 8). Wrote
`unitree_go2_sim/scripts/auto_stand.py`: opens a pty with Python's `pty` module, spawns
`ros2 run unitree_guide2 junior_ctrl` attached to it, waits ~3s for it to settle in
Passive, writes `2` (FixedStand), watches its own stdout for `percent=1.000` (with a 30s
timeout fallback), writes `5` (MOVE_BASE), then just relays output forever. Installed as a
proper package executable (`install(PROGRAMS ...)` in `unitree_go2_sim/CMakeLists.txt`).
Combined with the base sim + a `pointcloud_to_laserscan` node into one new launch file,
`unitree_go2_sim/launch/unitree_go2_nav_bringup.launch.py` -- confirmed this fully
automated sequence (spawn → stand → walking-ready) works with zero human keystrokes,
repeatably, across every restart in this session.

### Two more real bugs found and fixed at the source
1. **`odom→base_link` was never on TF at all** -- only the `/odom` *topic* worked. The
   `gz::sim::systems::OdometryPublisher` plugin in `unitree_go2_gazebo.xacro` had no
   `<tf_topic>` element; unlike `go2_simulation`'s own plugin config (which explicitly sets
   one), omitting it apparently means no TF output at all, not a sane default. Confirmed by
   watching `/tf` for several seconds and never once seeing a `base_link` child frame.
   Silent -- Nav2/SLAM would have just hung on `canTransform` forever with no obvious cause.
   Fixed by adding `<tf_topic>/tf</tf_topic>` to that plugin block.
2. Hit the exact same `robot_description`-as-YAML launch crash that
   `Command(...)` without `ParameterValue(..., value_type=str)` can cause (same class of
   bug as elsewhere in this project) -- fixed the same way in `unitree_go2_launch.py`.
3. (Our own bug, not the package's) `mapper_params_junior.yaml` set
   `correlation_search_space_smear_deviation: 0.15`, outside slam_toolbox's valid `[0.005,
   0.1]` range -- it crashed on startup (`std::runtime_error`) every time. Caught because
   the crash is loud, unlike the two above. Fixed to `0.1`.

### New config (`go2_navigation`, `go2_semantic_map`)
- `config/mapper_params_junior.yaml`, `launch/slam_junior.launch.py`: `base_frame: base_link`
  (this robot has no `base_footprint`), `scan_topic: /scan`, `min_laser_range: 0.5` (matches
  the Velodyne's own range floor).
- `config/nav2_params_junior.yaml`, `launch/nav2_junior.launch.py`: same structure as
  `nav2_params.yaml`, `base_frame_id`/`robot_base_frame: base_link` throughout, and
  **deliberately conservative velocity limits** (`max_vel_x: 0.25`, `max_vel_theta: 0.6` vs.
  the sliding robot's 0.5/1.0) given the balance controller's proven instability -- Nav2
  shouldn't push it harder than what's already been seen to (mostly) work.
- `maps/junior_world_map.{pgm,yaml}`: saved from a **rotation-only** mapping pass (see
  below for why) on `unitree_go2_description`'s bundled `default.sdf` world (open ground
  plane + 5 scattered props: `box1`/`box2`/`box3`/`cylinder1`/`cylinder2`, 4-7m from spawn,
  no walls -- so the map is a sparse "free-space rays from one vantage point" pattern, not
  an enclosed room like `go2_simulation`'s test world).
- `go2_semantic_map/config/places_junior.yaml`: **deliberately only 2 places**, both real
  tested points near the origin (`start` at spawn, `waypoint` at the exact goal the
  successful `NavigateToPose` run below completed to) -- not the actual `box`/`cylinder`
  landmarks, which are outside the safely-explored area. `go2_mcp_server` now reads
  `GO2_PLACES_FILE` (env var, absolute path) to pick a semantic map at runtime instead of
  always using the default `places.yaml`; no other MCP/LLM code changes were needed --
  `navigate_to_place`/`get_navigation_status`/`cancel_navigation` are Nav2-instance-agnostic
  by construction.

### What actually limits this: walking stability, not wiring
Mapping attempts hit the Milestone 8 balance-controller instability repeatedly and
concretely, not as a vague caveat:
- A full exploration pass (rotate + 4 translate/rotate legs, mirroring the pattern that
  worked fine on the sliding robot) fell partway through, even though each individual leg
  used speeds already proven safe in Milestone 8 (≤0.2 m/s).
- A second attempt, deliberately gentler (0.15 m/s straight-line, single leg), **also
  fell** -- ruling out "just go slower" as a fix; whatever's unstable isn't purely a speed
  function.
- **In-place rotation alone has never once failed**, across many repeats -- it's
  specifically translation (any observed magnitude) that's the risk.
- Consequence: the saved map only covers what a single rotate-in-place vantage point can
  see. Good enough to test Nav2 close to the origin; not enough to reach the world's actual
  landmarks.

Despite that, a real end-to-end goal succeeded cleanly: `NavigateToPose` to (0.9, 0.6) from
near the origin -- AMCL localized, the planner/controller drove `/cmd_vel`, `junior_ctrl`
walked it there, **`SUCCEEDED`**, orientation stayed near-identity (roll/pitch ~0) for the
entire ~20s run, one minor recovery along the way. Confirmed with live position/orientation
reads before and after, not just the status string.

A second, immediately-following test (this time through the actual LLM: *"go to the
start"*) surfaced two separate findings worth keeping distinct:
1. **A real, reproducible LLM failure mode**: `llama3.2:3b` called `list_places`, read back
   both place names/descriptions correctly, then **never called `navigate_to_place`** and
   just replied "navigation has started" as plain text -- confirmed by checking the live
   AMCL pose immediately after, which hadn't moved at all. A hallucinated tool call, not
   attempted. Worth remembering when picking/tuning a model for this role; not something to
   patch reactively here.
2. **The underlying tool logic still works correctly** -- called directly (bypassing the
   LLM) with the identical `navigate_to_place("start")`, it produced a real, tracked goal
   (`distance_remaining` genuinely decreasing at first), but this time the robot got stuck
   oscillating near the goal with a climbing recovery count and ultimately fell (confirmed
   via a large-pitch `/odom` orientation reading afterward) -- the same balance-controller
   limitation as above, not a new bug.

### Honest bottom line (as of Session 7 — see Session 8 update just below, this turned out to be fixable)
The wiring itself is done and proven: real walking → `/scan` → SLAM → a real map →
AMCL/Nav2 → a real, upright, successful autonomous goal, and the MCP/LLM layer needed zero
code changes to work against this robot (only a places-file swap). What's not done, and
was never in scope for a wiring task: making `junior_ctrl`'s balance controller actually
stable under sustained walking. That's the same open item Milestone 8 already flagged
(tune/replace the balance controller), now with much more concrete evidence of exactly
when it fails (translation, not rotation; not simply a matter of commanding lower speed).

### Session 8 update: the instability was a broken GPU driver, not the controller

User pushed back on "balance controller needs tuning/replacing" as the conclusion —
`junior_ctrl` had run smoothly for them before on a different machine, and they suspected a
**simulation timing problem** instead, and specifically asked to verify the GPU was actually
working. Both suspicions were exactly right:

- **`nvidia-smi` was failing** the entire time (`Driver/library version mismatch`): this
  machine's NVIDIA driver package (GTX 1650 Ti Mobile) had been upgraded via apt
  (580.126.09 → 580.173.02) at some point *after* the machine's last boot, so the resident
  kernel module was still the old version while the userspace libraries were the new one —
  DKMS already had the matching module built and ready, it just needed a reboot to load it.
  This forced every Gazebo run in Sessions 1-7 onto CPU software rendering
  (`LIBGL_ALWAYS_SOFTWARE=1`), which was already known/documented as a workaround for
  headless operation, but its *performance* cost had not been measured.
- **Measured real-time factor while software-rendering (Session 7 conditions): ~0.76** —
  the simulation was running at only 76% of real time. `junior_ctrl`'s balance/gait loop
  runs at a fixed 500Hz paced by wall-clock time (`ctrlComp->dt = 0.002`, not synced to
  `/clock`), so a simulation that's meaningfully behind real-time means the controller is
  computing corrections against a physics state that hasn't actually advanced as far as it
  assumes — exactly the kind of mismatch that destabilizes a real-time-tuned controller,
  independent of whether the controller itself is well-tuned.
- **After the user rebooted**: `nvidia-smi` succeeds, kernel module and driver package both
  580.173.02. Hardware-accelerated OpenGL confirmed working (`glxinfo`: `direct rendering:
  Yes`, `Accelerated: yes` — previously even software rendering was fragile/erroring).
  Relaunched the exact same `unitree_go2_nav_bringup.launch.py`, this time **without**
  `LIBGL_ALWAYS_SOFTWARE`/`MESA_GL_VERSION_OVERRIDE`. **Measured RTF: ~0.997** — essentially
  real-time, up from 0.76.
- **Redid every walking test that had previously fallen, back to back, same commanded
  speeds**: 8s sustained straight-line walk at 0.2 m/s (the exact speed that fell twice
  before) — stayed upright, moved a real ~1.9m. The full multi-leg
  rotate/translate/rotate/translate/rotate exploration sequence that fell partway through
  in Session 7 — completed multiple times in a row with orientation staying near-identity
  throughout (`w` consistently >0.94, `x`/`y` components consistently under ~0.02). A final
  18s in-place rotation (already known-safe, retested for completeness) — also clean.
  Every check used live `/odom` position and orientation, not a trusted status string.

**Conclusion: fixed, but calibrated — not "never fails again."** The repeated, reliable
falls from Milestone 8/9 were caused by this machine's broken GPU driver forcing an
effectively-non-real-time simulation against a real-time-paced controller, not a deficiency
in `junior_ctrl`'s balance control itself — no code changes were needed, only the
system-level driver fix. Went further after the initial confirmation: pushed to a higher
speed (0.35 m/s sustained, also stayed upright) and ran a full, real translation-heavy
exploration pass to actually map the world (not just the old rotation-only pass). That
longer session **did produce one fall** (same `SAFETY TRIP` signature, position not
obviously on top of any obstacle) after many minutes of continuous driving — so the honest
characterization is "dramatically more reliable, not provably perfect": every short,
targeted retest of a previously-100%-failing maneuver passed cleanly and repeatably, but a
single fall did occur during extended cumulative use. Updated
`nav2_params_junior.yaml`'s velocity limits to the values actually re-tested-safe
(`max_vel_x: 0.35`, was 0.25) rather than assuming full parity with `go2_simulation`'s 0.5.
Redid the SLAM mapping pass with real translation now that it's safe enough to attempt —
`junior_world_map` is now a real explored map (multiple obstacle features resolved: both
cylinders, two box corners) instead of the old single-vantage-point rotation-only sparsity.

---

## Key files

- `run_go2_demo.sh`, `RUN_DEMO.md` — one-command launcher + user-facing "how to run it"
  guide (Session 9). Start here if you just want to use the thing.
- `src/go2_simulation/urdf/go2.urdf` — robot description + all Gazebo sensor/plugin tags
  (see inline comments for the gravity/tipping history above)
- `src/go2_simulation/worlds/go2_world.sdf` — test room
- `src/go2_simulation/config/gz_bridge.yaml` — ROS↔gz-transport topic bridge
- `src/go2_simulation/launch/sim.launch.py` — brings up gz-sim + robot + bridge + `pointcloud_to_laserscan`
- `src/go2_navigation/config/mapper_params_online_async.yaml` — slam_toolbox params
- `src/go2_navigation/config/nav2_params.yaml` — AMCL/Nav2 params (see tuning history above)
- `src/go2_navigation/launch/slam.launch.py`, `launch/nav2.launch.py`
- `src/go2_navigation/maps/go2_world_map.{pgm,yaml}` — current saved map
- `src/go2_semantic_map/config/places.yaml` — named place → approach pose (edit to add places)
- `src/go2_semantic_map/go2_semantic_map/places.py` — loader (`load_places`, `yaw_to_quaternion`)
- `src/go2_mcp_server/go2_mcp_server/nav2_client.py` — Nav2 `NavigateToPose` action-client wrapper
- `src/go2_mcp_server/go2_mcp_server/server.py` — the 4 MCP tools + `main()`
- `unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy/unitree_go2_description/urdf/unitree_go2_gazebo.xacro`
  — has the `tf_topic` fix (Milestone 9)
- `unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy/unitree_go2_sim/launch/unitree_go2_launch.py`
  — has the `GZ_SIM_SYSTEM_PLUGIN_PATH` fix (Milestone 8) and `ParameterValue` fix (Milestone 9)
- `unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy/unitree_go2_sim/launch/unitree_go2_nav_bringup.launch.py`
  — sim + `/scan` + `auto_stand.py`, all in one launch (Milestone 9)
- `unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy/unitree_go2_sim/scripts/auto_stand.py` —
  automates Passive → FixedStand → MOVE_BASE (Milestone 9)
- `src/go2_navigation/config/{mapper_params_junior,nav2_params_junior}.yaml`,
  `launch/{slam_junior,nav2_junior}.launch.py`, `maps/junior_world_map.{pgm,yaml}` —
  junior_ctrl-specific SLAM/Nav2 config (Milestone 9)
- `src/go2_semantic_map/config/places_junior.yaml` — junior_ctrl-specific semantic map
  (Milestone 9)

## Useful commands

```bash
# Terminal 1 — simulation
source /opt/ros/humble/setup.bash && source /home/kan/lab/course/install/setup.bash
ros2 launch go2_simulation sim.launch.py

# Terminal 2 — EITHER mapping (slam_toolbox) OR localization+nav (Nav2), never both at once
# (both try to own the map->odom TF):
ros2 launch go2_navigation slam.launch.py
# ...drive around, then: ros2 run nav2_map_server map_saver_cli -f <path> --ros-args -p save_map_timeout:=5.0
# or:
ros2 launch go2_navigation nav2.launch.py
# ...then set /initialpose and send a NavigateToPose goal

# Terminal 3 — once Nav2 is up and localized, the MCP server (needs: pip3 install mcp)
ros2 run go2_mcp_server go2_mcp_server
# point any MCP client (Claude Desktop config, an agent framework, mcp.ClientSession, ...)
# at this command (or the installed path directly, see Milestone 6 notes) over stdio.

# Terminal 4 — the LLM front end (needs: pip3 install httpx, and a running Ollama --
# `~/.local/ollama/bin/ollama serve` if it's not already running -- with a model pulled,
# e.g. `ollama pull llama3.2:3b`). Spawns its own go2_mcp_server subprocess, don't also
# run Terminal 3's for this.
export PATH=/home/kan/.local/ollama/bin:$PATH
export GO2_LLM_MODEL=llama3.2:3b   # or unset for the default llama3.1:8b
ros2 run go2_llm_nav go2_llm_nav "go to the sofa"   # one-shot
ros2 run go2_llm_nav go2_llm_nav                    # interactive REPL

# The real-walking robot (Milestones 8-9), now wired to Nav2/SLAM/LLM. Its own separate
# workspace -- don't also run Terminal 1's go2_simulation, they're unrelated simulations
# and running both at once caused the load-24 CPU starvation documented in Session 6.

# Terminal A -- sim + /scan + auto-stand-to-MOVE_BASE, all in one, no keyboard needed:
source /opt/ros/humble/setup.bash
source /home/kan/lab/course/unitree_go2_ros2_jazzy1/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3   # same broken-driver workaround as go2_simulation
ros2 launch unitree_go2_sim unitree_go2_nav_bringup.launch.py
# wait for "Switched from fixed stand to move_base" in the log (~10-20s) before continuing.

# Terminal B -- EITHER mapping OR localization+nav, never both (same rule as Terminal 2):
source /opt/ros/humble/setup.bash && source /home/kan/lab/course/install/setup.bash
ros2 launch go2_navigation slam_junior.launch.py
# drive gently via /cmd_vel (ROTATION ONLY has been reliable -- any translation has a real
# chance of falling, see Milestone 9), then:
#   ros2 run nav2_map_server map_saver_cli -f <path> --ros-args -p save_map_timeout:=5.0
# or:
ros2 launch go2_navigation nav2_junior.launch.py
# ...then set /initialpose and send a NavigateToPose goal -- or use go2_llm_nav /
# go2_mcp_server exactly as in Terminal 3/4, with:
export GO2_PLACES_FILE=$(ros2 pkg prefix go2_semantic_map)/share/go2_semantic_map/config/places_junior.yaml

# To drive junior_ctrl by hand instead of through Nav2 (e.g. to test FixedStand/Trotting
# directly), it only accepts live keyboard input -- give it a real TTY:
screen -S junior ros2 run unitree_guide2 junior_ctrl
# inside: 2 (FixedStand), wait for percent=1.000, then 5 (MOVE_BASE, listens to /cmd_vel)
# or 4 (Trotting, listens to the keyboard directly) -- see Milestone 9 for what's stable.
```

Notes:
- `go2_semantic_map`/`go2_mcp_server`/`go2_llm_nav` are `ament_python` packages that
  currently fail `colcon build --symlink-install` (`error: option --editable not
  recognized`, a setuptools-version incompatibility) — build them with plain
  `colcon build` instead (no `--symlink-install`); re-run that whenever
  `places.yaml`/the Python source changes.
- Runtime Python deps not yet in any `package.xml` (pip-only, needs doing properly):
  `mcp`, `httpx`. Ollama itself is a separate user-local binary install, not a pip/apt
  package — see Milestone 7 for how it got installed on this machine (no sudo, no GPU).

---

## Changelog (most recent first)

### 2026-08-25 — Session 12: exploration MCP tools built + wired in; camera rendering is intermittently unreliable
- Built the MCP-side integration that Session 11 explicitly left as "not yet done":
  `go2_mcp_server/detection_store.py` (background-spinning accumulator subscribed to
  `/yolo/detections_3d`, dedupes by label+proximity within 0.75m, keeps
  highest-confidence position per object), `go2_mcp_server/map_view.py` (one-shot `/map`
  fetch downsampled to a coordinate-labeled ASCII grid, capped at 40 cells/axis so it
  fits an LLM prompt), and three new tools in `server.py`: `get_map_overview()`,
  `explore_to(x, y)` (arbitrary-coordinate navigation, the one deliberate exception to
  "never invent coordinates"), and `list_detected_objects()`. `navigate_to_place(name)`
  now also checks the detection store (case-insensitive/substring label match) when a
  name isn't a hand-authored place, so "go to the yellow cylinder" works whether that
  name came from `places.yaml` or was only ever found live. Detected-object navigation
  computes a standoff point (0.9m short of the object, along the line from the robot's
  current `/amcl_pose`) rather than sending Nav2 to the object's raw center -- otherwise
  this repeats the exact "goal inside the obstacle" bug found for the hand-authored
  table/pillar_2 place. Updated `go2_llm_nav/agent.py`'s system prompt with an explicit
  explore -> scan -> report workflow. Wired the whole thing into
  `run_go2_demo_junior.sh` as an automatic step (`GO2_DETECTION=off` to skip), including
  the EGL env vars and a default open-vocabulary class list.
- **Found and fixed a real bug during first live test:** `get_map_overview()` came back
  `"No /map received"` even though `/map` genuinely had a publisher and valid data.
  Root cause: `map_server` publishes `/map` once, latched (`TRANSIENT_LOCAL`
  durability) -- a subscriber using the default `VOLATILE` durability never receives
  that already-published message. Fixed by matching the QoS
  (`TRANSIENT_LOCAL`/`RELIABLE`, depth 1) in `map_view.py`'s subscription.
- Verified live (not just unit-level): `get_map_overview()` returned a real, correctly
  coordinate-labeled grid with obstacle clusters roughly matching known object
  positions; `explore_to(x, y)` produced genuine navigation (confirmed via `/odom`);
  `navigate_to_place("yellow_cylinder")` worked end-to-end through the full agent loop.
- **Camera rendering reliability is worse than Session 11 characterized it.** Session
  11's writeup treated the blank-frame bug as resolved by the EGL vendor override, with
  a caveat that it could recur *on launch*. This session found it can also **recur
  mid-session**, after the camera had already been working correctly for several
  minutes and successfully producing detections -- confirmed directly (pixel
  mean/std, not just "no crash") going from valid varied frames back to uniform
  blank (`mean=218, std=0`) with nothing else changed. Waited and rechecked twice;
  stayed blank both times before this session ended. This is a genuine, unresolved
  environment/graphics-driver reliability problem on this specific machine, not a bug
  in any of today's or yesterday's code -- `list_detected_objects()` returning empty
  when the camera has silently gone blank is indistinguishable, from the tool's
  perspective, from "genuinely nothing detected yet." **Not fixed this session** --
  flagging honestly rather than claiming it works reliably. A future session should
  consider: an MCP tool (or a check inside `list_detected_objects`) that reports camera
  frame health (e.g. pixel variance) so the agent/user can tell "nothing here" from
  "the camera silently broke," and/or investigating a lower-level fix for the
  underlying gz-sim offscreen-sensor-rendering flakiness itself.

### 2026-08-25 — Session 12 (cont'd): stop offering pre-known object names; crop the map; turn-in-place look_around()
User feedback after the first live test above: the agent answered "what do you see" by
offering the hand-authored object place names (`red_box`, `blue_box`, etc.) instead of
actually looking -- directly defeating the point of building live detection. Also
explicit: the agent should have *only* the map, no pre-baked object knowledge, and
should determine what's in the room (and its own position) live via the camera/YOLO
tools, so the person can later say "go to that object."
- **Removed the 5 hand-authored object places from `places_junior.yaml`** (`red_box`,
  `blue_box`, `green_box`, `yellow_cylinder`, `cyan_cylinder` -- added 2026-08-23/24
  while building the detection pipeline). Only `waypoint`/`start` remain as generic
  navigation reference points. Live-tested: the agent stopped offering those names and
  attempted real exploration instead.
- **Strengthened `agent.py`'s system prompt** from permissive ("you can explore on your
  own") to mandatory: explicitly forbids answering "nothing here"/"can't see anything"
  just because `list_detected_objects()` is currently empty -- an empty result means
  "haven't looked yet," not "room is empty."
- **Found and fixed a real bug during the live re-test of the two fixes above:** the
  agent picked an `explore_to()` point 7.9m away and burned its entire tool-call budget
  just waiting to arrive, never completing an explore -> detect -> report cycle (final
  reply: `"(gave up after too many tool-call rounds -- something's looping)"`). Root
  cause: `get_map_ascii()` rendered the *entire* saved map canvas, which `map_saver_cli`
  pads far larger than what's actually been explored (~38x24m file vs. ~10x10m actually
  seen) -- so "free" cells the agent could pick from included distant padding that was
  never really explored, just threshold-classified as free. Fixed in `map_view.py` by
  cropping the rendered grid to the bounding box of actual obstacle (`#`) pixels plus a
  3m margin (falling back to a fixed region around the origin if nothing's been
  explored yet).
- **Added, then replaced, a "look around" capability -- landed on a raw `rotate()`
  primitive instead of a canned macro.** First attempt was `look_around()`: a fixed
  full 2π turn-in-place (via Nav2's already-configured `Spin` behavior,
  `behavior_server`'s `spin` plugin, action `/spin`) that also auto-fetched position +
  detections and packaged them together. User explicitly rejected this: they want
  Gemini directing Nav2 itself off the plain unlabeled 2D map -- deciding e.g. how far
  to rotate, in what direction, whether to check detections between smaller increments
  or all at once -- not a single hardcoded predefined motion standing in for that
  judgment. **Replaced with `rotate(angle_deg)`**: turns the robot in place by
  whatever angle the agent picks (sign = direction), nothing else bundled in -- the
  agent calls `list_detected_objects()` separately whenever it wants to check what's
  been found. `Go2NavClient.spin()` (`nav2_client.py`) still does the underlying work
  and already took `target_yaw` as a parameter, so no change needed there -- blocks for
  the actual result (unlike `navigate_to_pose`, which only waits for goal acceptance
  since a drive can take minutes; a spin is short enough to just block). System prompt
  rewritten from a numbered step-by-step script to a description of the available
  primitives (`get_map_overview`, `rotate`, `explore_to`, `list_detected_objects`) with
  guidance but no mandated call order, so the agent plans the actual exploration
  itself. **Not yet live-tested** -- code complete, rebuilt, syntax-checked
  (`py_compile` + `colcon build --packages-select go2_mcp_server go2_llm_nav`), but the
  user is running the demo themselves (`./run_go2_demo_junior.sh`) rather than through
  this session, so the real spin-then-detect behavior is unverified. Also still
  unverified: whether Nav2's Spin behavior aborts if the robot starts very close to an
  obstacle (footprint collision check during rotation).
- **Follow-up correction: `explore_to()` was still too narrowly framed.** User pointed
  out Gemini should be able to move the robot wherever it wants in general, not just
  rotate in place -- `explore_to(x, y)` already did drive to any point, but its name
  and docstring ("for exploration, not a named place") boxed it in conceptually as an
  exploration-only tool. Renamed to **`navigate_to_point(x, y)`** and reframed as a
  general-purpose movement primitive (any reason: exploring, repositioning, getting
  closer to something, backing away) alongside `rotate()`, updated everywhere it's
  referenced (`server.py` module docstring + `navigate_to_place`'s error message +
  `list_detected_objects`'s docstring, `map_view.py`'s grid note, `agent.py`'s system
  prompt -- now has a "MOVING FREELY / LOOKING AROUND" section instead of a
  detection-only one, with rotate()/navigate_to_point() introduced as always-available
  primitives up front, not just inside the "what's in the room" flow). Rebuilt,
  syntax-checked. Still unverified live for the same reason as above.
- **User ran the demo themselves and hit a real, previously-undiscovered bug: every
  navigation goal was rejected (including the known `waypoint` place), and the initial
  camera scan found nothing.** Diagnosed via `/tmp/go2_demo_junior_logs/nav2.log`
  (readable from this session even though the demo's actual processes, launched in the
  user's own terminal, aren't visible to this session's `ps`/`kill` -- filesystem and
  the ROS/DDS network are shared, the process/PID namespace isn't). Root cause: AMCL
  repeatedly logged `"Received initialpose message is malformed. Rejecting."` --
  `nav2_util::validateMsg()` (confirmed by reading the actual navigation2 source,
  `humble` branch) requires a `PoseWithCovarianceStamped`'s orientation quaternion to
  have unit magnitude. `run_go2_demo_junior.sh`'s initial-pose step read `/odom`'s full
  orientation quaternion but only forwarded `.z`/`.w` (treating the robot as planar),
  dropping `.x`/`.y` without renormalizing -- fine for a locked-leg "sliding" robot with
  genuinely zero roll/pitch, but **wrong for junior_ctrl's real walking gait**, which
  has real roll/pitch sway, so the truncated quaternion's magnitude (`z^2+w^2`) came out
  `< 1`. AMCL silently rejected every initial-pose publish, so it never localized,
  `/map`'s transform never existed (`global_costmap` logged "Timed out waiting for
  transform from base_link to map to become available" repeatedly), and every
  `NavigateToPose` goal (from `navigate_to_place`/`navigate_to_point` alike) was
  rejected outright -- exactly matching what the user saw. Object detection itself was
  confirmed healthy in the same run's `yolo.log` (`yolo_node` activated, classes set
  successfully) -- the empty first scan was very likely a downstream effect (robot
  couldn't reach anywhere useful before the agent gave up), not a new camera bug.
  **Fixed** in both `run_go2_demo_junior.sh` and `run_go2_demo.sh` (same latent bug,
  hadn't manifested there yet but same fix applies): extract yaw properly via
  `atan2(2*(w*z+x*y), 1-2*(y*y+z*z))` on the *full* quaternion, then rebuild a clean
  normalized pure-yaw quaternion (`qz=sin(yaw/2)`, `qw=cos(yaw/2)`) instead of
  truncating. Syntax-checked (`bash -n` + `ast.parse` on the embedded Python heredoc).
  **Not yet live-verified** -- next run should confirm AMCL stops logging "malformed"
  and actually localizes/publishes the map transform.
- **User tried it live again: the robot fell while turning.** Diagnosed via
  `/tmp/go2_demo_junior_logs/sim.log`: `auto_stand.py`'s safety monitor was repeatedly
  logging `[SAFETY TRIP] tilt (rotMat(2,2)) = -0.615 < 0.5 (>60deg from vertical) --
  forcing passive` -- a genuine fall (not a false positive like the earlier
  8-consecutive-reading one in Session 10), stuck looping because nothing re-stands it
  automatically after a trip. **Root cause, found across three layers of the stack, all
  carrying the exact same unvalidated value:** `go2_navigation/config/nav2_params_junior.yaml`
  already had a comment (from the Session 8/9 GPU/RTF fix) explicitly stating "0.3 rad/s
  in-place rotation stayed upright too (**untested above that**)" -- yet
  `FollowPath.max_vel_theta` (DWB/controller_server), `behavior_server.spin.max_rotational_vel`
  (Nav2's Spin behavior, i.e. today's `rotate()` tool), and `velocity_smoother.max_velocity[2]`
  (the last stage before `/cmd_vel`) were all still set to **0.6 rad/s -- double the
  actually-tested-safe ceiling, and apparently never itself tested**. Nothing had
  exercised that untested value in practice until `rotate()` started calling Nav2's Spin
  behavior directly this session -- which promptly hit it and made the robot fall.
  Additionally, **the low-level `junior_ctrl` gait controller itself
  (`unitree_guide2`'s `State_move_base::twistCallback`) applied zero velocity clamping**
  -- it copied `/cmd_vel`'s `linear.x/y` and `angular.z` straight into the commanded
  gait velocity with no defense at all, so any upstream misconfiguration (this one, a
  stray teleop command, a future caller) would just be obeyed regardless of whether
  it's known-safe.
  **Fixed at all three points:**
  - `nav2_params_junior.yaml`: `FollowPath.max_vel_theta` 0.6->0.3,
    `acc/decel_lim_theta` 1.5/-1.5->0.75/-0.75; `behavior_server.spin.max_rotational_vel`
    0.6->0.3, `min_rotational_vel` 0.3->0.15, `rotational_acc_lim` 1.5->0.75;
    `velocity_smoother.max/min_velocity[2]` 0.6/-0.6->0.3/-0.3,
    `max/max_decel[2]` 1.5/-1.5->0.75/-0.75 -- all capped to the value the file's own
    comment says was actually validated.
  - `unitree_guide2/src/FSM/State_move_base.cpp` (both build variants,
    `COMPILE_WITH_MOVE_BASE` and the actually-compiled `COMPILE_WITH_ROS2_MB`):
    `twistCallback` now `std::clamp`s `linear.x/y` to +/-0.35 m/s and `angular.z` to
    +/-0.3 rad/s before storing them -- a hard backstop at the layer closest to the
    hardware/sim, independent of whatever Nav2 config says.
  Rebuilt both workspaces (`colcon build --packages-select unitree_guide2` in the
  junior workspace, `--packages-select go2_navigation` in the main one) -- clean,
  pre-existing warnings only.
- **User pushed back, reasonably: "did you just add clamps, is the controller actually
  stable, it was robust on a previous Ubuntu 24 system (straight walking AND turns)."**
  That's a fair challenge to "just cap the speed and call it fixed" -- worth actually
  checking rather than asserting. Investigated properly instead of just live-testing:
  - Found `FSM.cpp` has a **built-in loop-timing diagnostic** (`[loop timing]
    target=2000us avg_work=...us max_work=...us samples=...`, logged once/sec) that
    hadn't been noticed/used before. At the moment of the fall, `avg_work` was ~330us
    of a 2000us budget with ~478/500 samples/sec -- i.e. NOT the same signature as the
    earlier GPU-driver bug (which showed a gross RTF~0.76 collapse). The control
    process's own compute+IPC time was healthy.
  - Directly measured whether the newly-added YOLO-World inference contends for GPU
    with Gazebo enough to degrade real-time performance (a real possibility on this
    machine's modest laptop GPU, and the most likely mechanism if the "robust
    elsewhere" difference were environmental): `/clock` publish rate stayed ~963-968Hz
    with the sim idle-standing alone AND with YOLO-World fully active
    (GPU util 23%, 486MiB) -- no measurable degradation. Rules out GPU contention as
    the cause on this run.
  - **Directly reproduced the actual failure scenario and verified the fix under it**:
    manually brought up sim + YOLO-World + Nav2 (this session, standalone, not via the
    launcher script), applied the fixed initial-pose quaternion logic by hand,
    confirmed AMCL localized with zero "malformed" rejections, then sent a real
    `nav2_msgs/action/Spin` goal (`target_yaw: 6.283`, a full 2pi turn) through the
    actual `/spin` action server -- the same path `rotate()` uses. **Zero safety trips
    across the full rotation**; final `/odom` orientation was essentially flat
    (roll/pitch both <0.01 rad). Did notice occasional genuine loop-timing overruns in
    `sim.log` (`[WARNING] The waitTime=2000 of function absoluteWait is not enough!`,
    costs up to ~4453us against a 2000us budget) -- real jitter exists, just not
    severe/sustained enough to be an RTF collapse, and at the new 0.3 rad/s cap the
    robot had enough balance margin to absorb it. **Honest conclusion**: the fall
    wasn't purely an environment/timing bug (ruled that out directly) and wasn't purely
    a hard physical gait limit either (0.3 rad/s + real jitter stayed upright, so
    there's some margin, not a hair-trigger cliff at exactly 0.3) -- most likely the
    untested 0.6 rad/s simply had no margin left to absorb the jitter that does exist,
    where 0.3 rad/s does. Whether something between 0.3-0.6 would also hold wasn't
    tested (would need a careful incremental live probe) -- 0.3 rad/s is confirmed
    solid under the real failure conditions, that's where it stands.
  All test processes (manually-launched sim/YOLO/Nav2, separate from the user's own
  demo instance) cleaned up after.
- **Added walls to the world -- it was previously just an open plane.** User asked for
  "a more complex map...like a standard house or office" to test the model better.
  Scoped with the user first (AskUserQuestion): picked "partition current room" over a
  full hand-built house/office or importing a pre-built asset (no compatible
  house/office world exists locally for `ign gazebo`/gz-sim -- the one candidate found,
  `nav2_simple_commander`'s `warehouse.world`, is old Gazebo-Classic SDF referencing
  Fuel-hosted warehouse shelving models, not house/office, not verified compatible).
  Edited `unitree_go2_description/worlds/default.sdf`: added 4 exterior walls (1.2m
  tall, 0.15m thick) enclosing a ~12.5x12m rectangle around the 5 existing objects with
  real margin (~1m clearance to the nearest object edge), plus one interior dividing
  wall at y=-1 split into two segments leaving a 1.4m doorway near x=0 -- comfortably
  above 2x `robot_radius` (0.35m) plus inflation margin. Result: two connected rooms
  (3 objects north, 2 south) instead of one open area. Robot spawn point (2.0, 2.0,
  from `unitree_go2_launch.py`'s `world_init_x/y` defaults) confirmed clear of both the
  doorway and every wall/object. Hit (and fixed) the exact "--" in an XML comment
  gotcha already in memory ([[feedback-xml-comment-dashes]]) -- caught immediately via
  `python3 -c "import xml.etree.ElementTree..."` validation before ever launching sim.
  Rebuilt `unitree_go2_description`. **Live-verified, not just asserted**: launched the
  sim standalone, robot stood up cleanly ("now listening to /cmd_vel"), zero safety
  trips, and `/scan` returned real finite ranges (~2.9m) consistent with detecting the
  new walls from the spawn point -- confirms the world loads and physics are stable
  with the new geometry. Cleaned up after.
  **Critical follow-up required before this is actually usable**: `junior_world_map.pgm/yaml`
  (the saved SLAM map Nav2/AMCL/the LLM's `get_map_overview()` all rely on) predates
  these walls entirely and does not reflect them -- Nav2 will plan through walls that
  don't exist in its map, and AMCL will see laser hits the map can't explain. **A full
  re-mapping pass (drive the robot through both rooms and the doorway with
  `slam_junior.launch.py` running, then `map_saver_cli`) is required before the new
  layout is actually navigable** -- same category of work as Session 10's ~40min
  re-explore.
- All processes (sim, Nav2, YOLO, chat session) were shut down at the user's request;
  they're running the demo themselves going forward via `./run_go2_demo_junior.sh`.

### 2026-08-31 — Session 13: project cleanup/docs, git catastrophe + recovery, capability-gap tools

**Cleanup and documentation.** User asked to organize the project (keep only needed
files, cleanly structured) and write an installation guide, usage guide, and project
summary. Given the project had **no version control at all** up to this point, set up
git first as a safety net before touching anything -- `git init` at the root, plus a
safety-baseline commit inside `unitree_go2_ros2_jazzy` (which already had its own git
history and 219 files of uncommitted hand-edited work: the walled `default.sdf`,
`rgbd_camera` xacro changes, the velocity clamps, the whole `yolo_ros` clone -- none of
it protected before this). Removed genuine cruft (`__pycache__`, a stray nested colcon
build tree inside `go2_navigation/maps/`, ~380MB of duplicate YOLO-World weights,
verified via CWD-resolution reasoning + md5 before deleting). Kept `unitree_ros2/`
(the real-hardware DDS bridge) on the user's explicit call -- not simulation cruft,
deliberately preserved for eventual physical-robot use. Wrote `README.md` (project
summary + architecture diagram), `INSTALL.md` (step-by-step setup), `USAGE.md`
(replaces the stale `RUN_DEMO.md`, updated for the current walled-world/no-pre-baked-
objects state). Caught and fixed a self-inflicted regression along the way: deleted
`src/go2_simulation/rviz/` as "empty cruft," which broke the build (`CMakeLists.txt`
installs from it, needs the directory to exist even empty) -- restored with a
`.gitkeep`, and while investigating found a real pre-existing footgun: a bare
`colcon build` from the project root recursively tries to build the two nested
workspaces too and fails; documented `colcon build --base-paths src` as the fix rather
than papering over it with ignore markers (tried those first, reverted -- they also
break building the nested workspace from within itself, which is the documented way
to do it).

**Real-hardware tangent.** User asked whether this could run on the real Go2, and
separately reported the phone app's 2D map wasn't visible over ROS after connecting via
Ethernet with topics "mostly empty." Diagnosed from `unitree_ros2`'s own docs/scripts
(no live hardware access this session): the app's SLAM/mapping is part of Unitree's own
closed onboard stack, not exposed over the SDK2/DDS bridge at all (the bridge only
exposes raw sensor/control topics -- `/utlidar/cloud`, IMU, `/lowstate`/`/lowcmd`,
`/sportmodestate` -- no `/map`). Separately found and fixed a real bug in
`unitree_ros2/setup.sh`: the line sourcing the built Unitree message-package overlay
(needed for ROS 2 to understand those topics at all) was commented out and pointed at a
path (`cyclonedds_ws/install/setup.bash`) that doesn't exist in this checkout -- the
real overlay is at `install/setup.bash` (this workspace was built directly in
`unitree_ros2/`, not via the README's `cyclonedds_ws` layout). Verified the fix:
`ros2 pkg list`/`ros2 interface list` correctly show `unitree_go`/`unitree_api` after
sourcing it. This plausibly explains the "mostly empty" topics report, though it
couldn't be confirmed against the actual physical robot this session.

**Git catastrophe and recovery.** Mid-session, discovered `src/`, `PROGRESS.md`,
`README.md`/`INSTALL.md`/`USAGE.md`, and both launcher scripts were entirely missing
from disk -- the user had independently deleted the git history this session created
and rebuilt it fresh, then pushed to GitHub (`mak1711/course`). The fresh "Initial
commit" that got pushed turned out to only contain `build/`/`install/`/`log/` (colcon
artifacts) plus the two nested workspaces -- the actual source, docs, and scripts never
made it into that commit (most likely deleted from disk in the same cleanup pass,
before the fresh `git add`/commit). Recovered cleanly: this session's own prior commit
was still reachable via the reflog (git doesn't garbage-collect immediately), diffed
byte-for-byte clean against it, restored the missing paths, and committed the recovery
-- all without any destructive git operation (no reset --hard, no force-push, purely
additive). Separately discovered both nested repos (`unitree_go2_ros2_jazzy`,
`unitree_ros2`) lost their own independent `.git` in the same event -- their commit
*history* is gone, but the actual file *content* was verified intact on disk (checked
the velocity-clamp fix and the walled world directly). Per the user's explicit
instruction, folded both into the main repo as regular tracked files going forward
(confirmed via `git ls-tree` -- real tree objects, not gitlinks) rather than
re-establishing them as separate embedded repos. **Lesson worth keeping**: this
session's own "Initial commit"/fresh `git init` from the user's side didn't respect
the `.gitignore` already in place, re-tracking `build`/`install`/`log`/`__pycache__`
(3407 files) across all three repos -- had to `git rm --cached` all of it again while
fixing the above; .gitignore only blocks *new* untracked files, it does nothing for
files a fresh `git add -A` already picked up.

**Capability-gap tools.** Earlier discussion (why not use a ROSA-style broad-access
agent) concluded the narrow, typed-tool approach here is the right call for a system
actually moving a physical robot -- but identified three real, worth-fixing gaps in the
current narrow tool set. Fixed all three:
- **`set_detection_classes(classes)`** (`go2_mcp_server`): wraps YOLO-World's
  `/yolo/set_classes` service so the agent can expand its own search vocabulary at
  runtime instead of being stuck with whatever was set at launch. Not live-tested
  end-to-end (needs YOLO-World running), but reuses the exact `call_async`/poll-future
  pattern already proven elsewhere in `detection_store.py`.
- **`list_ros_nodes()`** and **`echo_topic(topic_name)`**: broader read-only
  introspection, the "ROSA-style breadth without the action-layer risk" middle ground
  discussed earlier -- `echo_topic` resolves any topic's message type dynamically via
  `rosidl_runtime_py` (the same mechanism `ros2 topic echo` itself uses under the
  hood), not hardcoded to one type. **Live-tested**: a throwaway
  publisher/subscriber pair confirmed dynamic type resolution and message
  serialization both work correctly.
- **Raised the LLM tool-call round cap from 8 to 20** (`go2_llm_nav/agent.py`) -- the
  agent now has enough legitimate multi-step tools (explore cycles, introspection,
  vocabulary changes) that 8 could plausibly cut off a longer task partway through.

**Full end-to-end verification, prompted by being asked directly "did you test the
full system."** Honest answer at that point was no -- the new tools had only been
build-checked and, for two of three, tested via direct Python calls, not through the
actual MCP protocol or the real demo stack. Did the real thing instead of just
reporting the gap:
- Full clean rebuild of both workspaces from scratch (`rm -rf build install log` +
  `colcon build`) -- 5/5 and 8/8 packages, no errors.
- Launched the real junior demo stack (sim + Nav2 + YOLO-World), set the initial pose,
  confirmed AMCL localized.
- Tested every new tool through the **actual MCP protocol** (spawned `go2_mcp_server`
  as a real subprocess via `stdio_client`, exactly as `go2_llm_nav` does -- not direct
  Python function calls): `list_ros_nodes`, `echo_topic("/odom")`, and
  `set_detection_classes` all confirmed working with real data.
- **`set_detection_classes` failed on the first attempt** (timed out) -- root-caused
  properly rather than assumed: `ros2 service call` directly against `/yolo/set_classes`
  showed the *first* call triggers `yolo_node` downloading an additional ~338MB CLIP
  resource on demand (~20-25s one-time cost); a second immediate call took ~100ms. The
  10s default timeout was just too short for a cold start. Fixed (45s default,
  documented why in the docstring), rebuilt, re-verified: succeeds cleanly now.
- Also re-verified the **pre-existing** core navigation still works, not just the new
  tools: `navigate_to_place("waypoint")` through the same real MCP protocol, watched
  `/odom` move from the origin toward the target (0.79, 0.42) over several seconds,
  zero safety trips.
- Cleaned up afterward -- again had to kill several processes by exact PID since
  `pkill` patterns missed them (the same `ros2 launch` process-tree gotcha from
  earlier this session; see [[feedback-bash-harness-gotchas]]).

### 2026-08-26 — Session 12 (cont'd): re-mapping the walled world
User ran the demo themselves against the new walled world before the re-map above was
done, and hit exactly the predicted failure: `nav2.log` showed `"Begin navigating from
current location (-0.12, -0.25) to (0.30, -3.83)"` -- a goal straight from the north
room to the south room, planned on a map that still thought that whole area was open.
The robot physically hit the new wall and fell (`rotMat(2,2) = -0.222`, stuck for the
rest of the run -- 3887 repeated safety trips). Confirmed via log evidence *before*
touching any code that this was not a new controller bug -- the velocity-clamp fix from
earlier this session was never wrong, it just doesn't help against colliding with an
obstacle the map doesn't know exists. User then asked to redo the SLAM map (raw 2D grid
only, no semantics -- reaffirming the standing requirement) and floated RTAB-Map as an
alternative worth considering. Scoped via AskUserQuestion: picked the already-proven
`slam_toolbox` pipeline over RTAB-Map (RTAB-Map's visual/depth path would inherit the
still-unresolved intermittent camera issue; a lidar-only RTAB-Map setup would still be
new, unproven integration work with no clear win over what's already working here).

- Wrote a scripted go-to-point exploration (`/odom`-feedback P-controller, 0.2 m/s
  linear / 0.25 rad/s angular -- both under the tested-safe ceilings established
  earlier this session, see [[project-go2-walking-fixed]]) to drive both rooms and the
  doorway.
- **First attempt fell.** The naive hand-picked route had NO obstacle avoidance and a
  straight-line leg cut directly past `box2`; the robot physically collided with it
  (`rotMat(2,2)=0.476`, marginal but real -- final position stuck right next to the
  box). Root-caused with a small Python clearance checker before trying again, which
  also caught that several *other* hand-picked segments would have clipped `box1`,
  `box3`, and `cylinder2` too -- the one that actually failed wasn't even the worst
  offender, just the first one reached.
- **Rebuilt the route properly**: generated a grid of candidate waypoints each
  individually clear of every object/wall by `robot_radius` + margin, built a
  visibility graph (every edge checked for object/wall clearance AND for illegally
  crossing the solid part of the interior wall outside the 1.4m doorway band), and took
  a Dijkstra shortest path through it between must-visit targets -- 17 points total,
  every single segment pre-verified collision-free instead of eyeballed.
- Also hit, and correctly identified as separate, known, unrelated flakiness: an
  **intermittent sim-startup race** -- one launch fell spontaneously at rest, zero
  commanded velocity, immediately after the passive->fixed-stand->move_base transition.
  No loop-timing anomaly, no `/cmd_vel` involved at all. A clean relaunch of the exact
  same code stood up fine and stayed stable; treated as the already-documented
  launch-time race ([[project-go2-walking-fixed]]), not a new bug -- and it didn't
  recur.
- Ran the validated 17-point route twice (**zero falls across both full laps**). The
  first `map_saver_cli` save showed most walls/objects reading as "unknown" rather than
  occupied: `mapper_params_junior.yaml`'s `min_pass_through: 2` means a cell needs 2+
  qualifying scan hits before slam_toolbox commits it to occupied, and a single sparse
  pass near most surfaces wasn't enough (the north wall, visited close-by from two
  separate waypoints, came through fine even on lap one). The second lap fixed
  everything except two exterior walls -- when checked with the same single-pixel
  sampling method.
- **That last "gap" turned out to be a verification bug, not a mapping bug.** Sampling
  exactly on an object's or wall's centerline coordinate reads "unknown" because that's
  the *interior* -- physically unreachable by any external lidar ray (the same
  "surface offset, not exact center" behavior already noted for this world in
  [[project-go2-junior-places]]). Re-checked by scanning a small neighborhood around
  each target instead of one exact point: every wall and every object showed a real
  `value=0` (confidently occupied) cluster nearby. The map is genuinely good.
- Installed to `go2_navigation/maps/junior_world_map.{pgm,yaml}`, rebuilt
  `go2_navigation`. All exploration/SLAM/sim processes killed and verified gone at the
  process level. Two process-management lessons worth keeping: (1) `ros2 node list`
  lagged behind actual process death by several seconds on stale DDS discovery more
  than once this session -- `ps aux` is the authoritative check, the ROS graph isn't;
  (2) killing a `ros2 launch` process is not the same as killing its process tree --
  `robot_state_publisher`/`parameter_bridge`/`junior_ctrl`/`slam_toolbox` etc. kept
  running as orphaned children after the launch PID died, and had to be killed
  individually by PID. `run_go2_demo_junior.sh` already handles this correctly via
  `pkill -f` patterns for exactly this reason; ad hoc manual cleanup during this
  investigation didn't, twice, and briefly produced a confusing multi-instance false
  alarm (duplicate `/estimator`, `/state_mb`, `/slam_toolbox` nodes from overlapping
  launches) before being caught.
- **Not yet tested against Nav2/the full demo.** The map itself is verified correct at
  the pixel level, but actually running Nav2 (localizing and navigating against it) and
  the LLM agent (`rotate()`/`navigate_to_point()`/detection) against the new map hasn't
  happened yet -- that's the next real end-to-end test.

### 2026-08-24 — Session 11: real object detection (yolo_ros + YOLO-World + RGB-D)
- User asked for a fundamentally different way to define places: instead of hand-authored
  `places.yaml` coordinates, have the robot actually *see* the room via camera + object
  detection and locate things on its own. Pointed at
  [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) as a candidate.
- Scoped it first rather than diving in: junior robot only (already has an RGB camera to
  upgrade, `go2_simulation` has none), and confirmed with the user that standard
  COCO-trained YOLO would not recognize this world's plain colored primitives
  (`box1..3`, `cylinder1..2` in `default.sdf`) as anything meaningful -- picked
  **YOLO-World** (open-vocabulary, text-prompted classes) over adding realistic 3D
  furniture models.
- **Built, in order:**
  - Junior's RGB camera sensor (`unitree_go2_gazebo.xacro`) changed from `type="camera"`
    to `type="rgbd_camera"` -- one attribute, gets a real depth image for free. Bridged
    the three new gz topics (`image`, `depth_image`, `camera_info`) into ROS via
    `unitree_go2_launch.py`'s `gazebo_bridge`.
  - Cloned `yolo_ros` (Humble-compatible) into the junior workspace's `src/`, built with
    `rosdep install` + `colcon build` (`yolo_msgs`, `yolo_ros`, `yolo_bringup`) --
    all ROS-side deps already satisfied via apt.
  - `pip install --user ultralytics` (already had `torch` with CUDA from earlier work)
    plus `lap` and (later) `clip` -- see gotchas below.
- **Five real, non-obvious bugs found and fixed, in the order discovered (each one hid
  the next):**
  1. **Blank camera frames.** New depth topics existed and looked normal size-wise, but
     every pixel was a uniform flat value (`min==max==218`, depth all `-inf`) -- not
     "nothing in view," an actual broken render. Root cause: `libEGL warning: egl:
     failed to create dri2 screen` in the gz-sim log -- the sensor's *offscreen*
     rendering thread (separate code path from the interactive GUI window, which is why
     everything looked fine visually all session) was failing to get a GPU-backed EGL
     context on this NVIDIA-hybrid-graphics laptop. Fixed by forcing the NVIDIA EGL
     vendor explicitly: `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`,
     `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, set before
     launching the sim. Confirmed by inspecting actual pixel data (`cv_bridge` +
     `numpy`), not just "no crash" -- saved and viewed a frame to see the red box for
     real. **Caveat: this specific failure turned out to be an intermittent sim-startup
     race, not fully eliminated by the env vars** -- it recurred on a subsequent fresh
     launch with the exact same fix already applied, then worked again on a retry. If a
     future session sees blank/uniform camera frames, check pixel variance directly
     (mean/std, not just topic-exists) before trusting the image, and just restart the
     sim if it's flat -- don't assume the fix regressed.
  2. **`uv sync` re-downloading ~2.5GB on every launch.** `yolo_bringup`'s launch file
     runs `uv sync` unconditionally to build an isolated venv, even though a working
     `torch`+CUDA+`ultralytics` already existed at the system Python level. Patched
     `yolo_bringup/launch/yolo.launch.py`'s `run_yolo()` to skip the `uv sync` call and
     just use the existing `PYTHONPATH` -- installed the one missing piece (`lap`) with
     plain `pip install --user` instead.
  3. **`No module named 'clip'`.** YOLO-World's text encoder needs OpenAI's CLIP
     package, which ultralytics normally auto-installs on first use via
     `pip install git+https://github.com/ultralytics/CLIP.git` -- that auto-install
     silently failed inside the running node. Installed it directly with the same pip
     command.
  4. **Detections never cleared the confidence threshold.** Standalone testing (loading
     a saved frame directly into `ultralytics.YOLO`, bypassing ROS) showed YOLO-World
     *does* find the right object and region, just at very low confidence (2-12%) --
     flat-shaded synthetic renders score far below real photos for CLIP-based matching.
     Two compounding issues: (a) `yolo_node`'s `threshold` parameter is read once at
     `on_configure`, not re-read on `ros2 param set` -- a runtime change silently does
     nothing, it must be passed as a launch argument (`threshold:=0.02` worked). (b)
     `detect_3d_node` divides depth by `depth_image_units_divisor` (default `1000`,
     assumes millimeters like a real depth camera) -- our simulated depth is already in
     **meters**, so this was scaling every reading down by 1000x. Fixed with
     `depth_image_units_divisor:=1`.
  5. **`/yolo/detections_3d` stayed empty even with 2D detections visibly working.**
     Added temporary debug logging directly into `detect_3d_node.py`'s
     `process_detections`/`convert_bb_to_3d` (removed afterward) to find exactly where
     detections were dropping. Root cause: `detect_3d_node` subscribes to `/yolo/tracking`
     (the ByteTrack tracker's *confirmed* output), not raw `/yolo/detections` --
     ByteTrack requires a detection to persist across several consecutive frames before
     confirming a track, and our noisy 2-12%-confidence detections never survived that.
     Fixed with `use_tracking:=False` at launch, which also changes
     `detect_3d_node`'s subscription to the raw (unfiltered) detections topic.
  - **Sixth bug, found after 3D detections started flowing:** the reconstructed 3D
    positions were nonsensical -- an object visibly in front of the robot came back
    with a near-zero forward (X) coordinate and a ~3m Z (height!). Root cause: depth
    images are inherently in the camera *optical* convention (Z=forward, X=right,
    Y=down), but `camera_link` had zero rotation relative to the robot body (X=forward,
    Z=up, like every other link) -- so `detect_3d_node`'s frame transform never
    corrected for the axis swap. Fixed the standard, correct way (REP-103): added a
    `camera_link_optical` child frame with the canonical `rpy="-pi/2 0 -pi/2"` rotation,
    and pointed the sensor's `gz_frame_id` at that instead of `camera_link` directly.
    Confirmed fixed: reconstructed heights came back as 0.36m/0.67m (plausible for
    these objects) instead of 2-3m.
- **Final working launch config**, for reference:
  ```
  ros2 launch yolo_bringup yolo-world.launch.py \
    input_image_topic:=/rgbd_camera/image \
    input_depth_topic:=/rgbd_camera/depth_image \
    input_depth_info_topic:=/rgbd_camera/camera_info \
    target_frame:=map \
    threshold:=0.02 \
    use_3d:=True \
    use_tracking:=False \
    depth_image_units_divisor:=1 \
    device:=cuda:0
  ```
  Classes set afterward via the `/yolo/set_classes` service (`yolo_msgs/srv/SetClasses`,
  a plain `string[] classes` request) -- this is what lets an LLM/agent pick the
  vocabulary at runtime rather than it being fixed at launch.
- **Not yet done** (this entry documents infrastructure only, not the end-to-end
  feature): no MCP tools exist yet to actually use this from the chat agent. Still
  needed: a way for the agent to see the known map (so it can pick sensible exploration
  points itself, per the user's request -- not a scripted/hardcoded exploration path
  and not full autonomous frontier exploration either), a tool to drive to an arbitrary
  (not named-place) coordinate, a detection accumulator (subscribe to
  `/yolo/detections_3d`, dedupe by proximity+label, store), a `list_detected_objects`
  tool, and either a new `navigate_to_detected_object` tool or teaching
  `navigate_to_place` to also check the detected-objects store. None of this is wired
  into `go2_mcp_server`/`go2_llm_nav` yet -- next session should start here.

### 2026-08-23 — Session 10 (cont'd): junior world re-mapped to all 5 landmarks + ROS introspection tool
- User asked why the real robot only had `start`/`waypoint` and wanted the agent to
  handle general questions like "what ROS topics are available." Clarified: the map
  itself has no semantic labels at all (a raw occupancy grid can't tell you "this is
  the sofa") -- `sofa`/`table` only exist for the other demo because a human wrote their
  coordinates in `places.yaml` to match furniture objects that happen to exist in that
  Gazebo world. `start`/`waypoint` being the only junior places wasn't a config gap --
  the junior world's other 5 objects (`box1/2/3`, `cylinder1/2`) had genuinely never
  been explored/mapped (left conservative in Session 9 after the walking controller
  fell during earlier extended-exploration attempts, pre-GPU-fix). User chose to add
  real furniture-equivalents and label them once, same pattern as the other demo.
- **Verified via pixel-level map inspection (not assumption) that this was really true**
  before doing anything: sampled all 5 object world-coordinates against the saved
  `junior_world_map.pgm` -- every one was `205` (unknown/unexplored), confirming they'd
  never actually been seen by the lidar.
- Re-explored: launched the real sim + SLAM (`slam_junior.launch.py`), wrote a scripted
  `/odom`-feedback go-to-point teleop (conservative 0.2 m/s, tested-safe this session)
  to drive toward each of the 5 objects and back to origin between legs. Included a
  tilt-based abort check for real fall safety.
- **Hit a real safety event, handled correctly, then fixed a false-positive:** the tilt
  check aborted mid-drive (`tilt=0.53 rad`) near the cylinder2 leg. Did NOT just resume
  blindly -- checked `/odom` orientation, `/joint_states`, and process health first.
  Findings: orientation was near-upright, joint angles matched a normal standing pose,
  velocities ~0 -- the robot was genuinely fine, just paused mid-gait-sway. This was a
  false positive from checking a single noisy odom sample against too tight a threshold
  (normal dynamic quadruped gait has real momentary body sway). Fixed by requiring 8
  consecutive over-threshold readings before treating it as a real tip, then resumed
  the remaining legs from where it left off. No actual fall occurred this session.
- Saved the new map (`map_saver_cli`), verified via the same pixel-sampling approach
  that all 5 objects now show real occupied clusters near their known SDF poses (object
  surface, not exact center -- expected). One small unexplained occupied cluster near
  (-2 to -3.5, -2.5 to -3.5) doesn't match any known object -- likely a transient
  self-detection artifact (lidar catching the robot's own legs during gait), left as a
  minor unresolved oddity, not blocking.
- Installed the new map into `go2_navigation/maps/junior_world_map.{pgm,yaml}`. Added 5
  new places to `places_junior.yaml` (`red_box`, `blue_box`, `green_box`,
  `yellow_cylinder`, `cyan_cylinder`), approach poses computed at ~1.2-1.3m from each
  object's known center (object half-extent 0.5m + robot_radius 0.35m + margin) --
  calculated, not guessed from the map image. Rebuilt `go2_navigation` and
  `go2_semantic_map`.
- **Verified live end-to-end**, not just via calculation: relaunched Nav2 on the new
  map, asked the agent to list places (all 7 showed up correctly), then "go to the
  yellow cylinder" -- watched `/odom` directly, robot walked out to ~(2.2, 1.8) (target
  was (2.1, 2.1)) and settled stably. Confirms the whole re-mapping and place-labeling
  actually works for real navigation, not just that the files parse.
- Added `list_ros_topics` to `go2_mcp_server` (a thin wrapper around
  `rclpy.Node.get_topic_names_and_types()` via the existing `Go2NavClient.node`) so the
  agent can answer general ROS-introspection questions instead of having no tool for
  them at all. Verified live: asked "what ros topics are available?", got a real,
  complete topic list back via the tool call, and Gemini summarized the ~90 topics into
  a sensible categorized answer rather than dumping them all.

### 2026-08-23 — Session 10 (cont'd): root-caused and fixed the table recovery-loop bug
- Followed up on the recovery-loop issue flagged (but not investigated) in the previous
  entry. User explicitly OK'd interrupting the live robot to investigate.
- **Root cause found via geometry, not trial-and-error:** `places.yaml`'s `table`
  approach pose was `(-1.8, -1.3)`, only **0.36m** from `pillar_2` at `(-1.5, -1.5)` in
  `go2_world.sdf`. Required clearance is `robot_radius (0.35) + pillar_radius (0.15) =
  0.5m` -- the goal itself sat inside the pillar's collision/inflation zone
  (`inflation_radius` 0.55 extends the costed region to 0.70m from the pillar). Nav2
  could never actually settle within `xy_goal_tolerance` (0.25m) of that point without
  entering costed/lethal space, which explains the exact observed symptom: 20+ recovery
  behaviors, `distance_remaining_m` stuck at a stale `0.0`, no real progress. Explains
  the intermittent nature too -- whether the final approach angle happened to graze the
  worst of the inflated zone or not would vary run to run.
- Fixed by moving the `table` place to `(-3.5, -2.0, yaw=0.0)` -- approaching from due
  west instead of the NE-ish diagonal that ran past the pillar. Verified clearances by
  calculation before touching anything: 2.06m from `pillar_2` (vs. 0.36m before), 0.6m
  from the table's own edge (comfortable margin above the 0.35m robot radius), 1.4m from
  the west wall.
- **Verified live by reproducing the exact original failure sequence, not just spot-
  checking:** navigated to sofa first (matching the original repro's starting
  condition), let it settle, then navigated to table -- watched `/odom` directly (not
  `get_navigation_status`, which turned out to be unreliable across separate single-
  prompt CLI invocations, each spawning its own `go2_mcp_server` with no memory of a
  previous invocation's goal -- a testing-methodology catch worth remembering, not a
  real bug). Robot settled cleanly at `(-3.53, -1.79)`, right at the new goal, stable
  across 16+ seconds. No recovery loop.
- **Incidentally found and fixed a second, unrelated bug while re-launching Nav2 for
  this test:** an earlier session's `robot_state_publisher`/`ros_gz_bridge` had survived
  as orphans after their parent `ign gazebo`/launch process died (window closed), and a
  fresh `sim.launch.py` launch alongside them produced duplicate ROS nodes -- this
  silently hung the next `nav2.launch.py` launch entirely (its `nav2_container` process
  started but never progressed past that single log line, confirmed via `ros2 component
  list` returning nothing and no per-node log files ever appearing). Killing the orphans
  and relaunching Nav2 fresh fixed it immediately. Not fully explained why the ROS graph
  confusion specifically manifests as a *silent hang* rather than a visible error, but
  reproducible enough (happened once, fixed the same way each time this pattern's been
  hit this session) to note as a troubleshooting step: if a fresh `nav2.launch.py` seems
  to hang forever with nothing past "process started" in its log, check for orphaned
  `robot_state_publisher`/`ros_gz_bridge` from an earlier sim instance before assuming a
  Nav2-specific bug.
- Rebuilt `go2_semantic_map`.

### 2026-08-23 — Session 10 (cont'd): native desktop window (`go2_llm_nav_gui`)
- User clarified the GUI request further: "not a website just a local window that opens
  and we interact with it" -- the browser-tab version from the previous entry wasn't
  what they meant.
- Checked what's available before adding anything: `pywebview` wasn't installed but
  pip-installed cleanly at the user level (`pip3 install --user pywebview`, no sudo);
  GTK3 + WebKit2GTK (pywebview's Linux backend) were already present.
- New `go2_llm_nav/desktop_ui.py` (console script `go2_llm_nav_gui`): runs the exact
  same aiohttp app from `web_ui.py` (imported, not duplicated) on a background thread
  with its own event loop, then opens it in a native `pywebview` window on the main
  thread -- no address bar, no browser chrome, just the chat. Window-close is wired to
  signal the background asyncio loop to shut down the MCP session/server cleanly via
  `loop.call_soon_threadsafe`.
- **Verified the window is actually real, not just "the process didn't crash":** this
  system runs Wayland (`XDG_SESSION_TYPE=wayland`), and GTK's native Wayland windows
  aren't visible to X11 inspection tools like `xwininfo` -- so a "silent success" here
  could just as easily have meant "invisible window" as "working." Confirmed the
  distinction directly: with `GDK_BACKEND=x11` forced, the exact same window showed up
  in `xwininfo -root -tree` as `"Go2 Navigator" 1100x750`, positioned on screen. Baked
  `os.environ.setdefault("GDK_BACKEND", "x11")` into `desktop_ui.py` itself (before
  pywebview's import) so this is automatic and doesn't rely on an unverified
  native-Wayland code path -- an advanced user can still override it by exporting
  `GDK_BACKEND` themselves first.
- `run_go2_demo.sh`/`run_go2_demo_junior.sh`: default hand-off changed again, this time
  to `go2_llm_nav_gui`. `GO2_UI=web` keeps the old browser-tab behavior, `GO2_UI=cli`
  the plain terminal chat.
- Verified the **full script**, not just the standalone GUI binary: ran
  `run_go2_demo.sh` end-to-end (sim -> Nav2 -> initial pose -> GUI window all came up in
  order), confirmed the window via `xwininfo`, sent a real navigation prompt through it
  (`/api/chat` -> `navigate_to_place` -> `ok: true`), then stopped the whole thing with
  `SIGINT` (matching how a user would Ctrl-C it) and confirmed the script's cleanup trap
  actually tore down every process this time -- validates the Session 10 cleanup-pattern
  fix from earlier today held up under a realistic full run, not just the piece it was
  originally tested against.

### 2026-08-23 — Session 10 (cont'd): browser chat GUI (`go2_llm_nav_web`)
- User asked for "a user interface... like a gui" instead of the terminal chat.
- Refactored `agent.py` slightly to support this without duplicating logic: `run_agent`
  now takes an optional `on_event` callback (called for every tool call/result) instead
  of hardcoding `print()`, and pulled the API-key check and MCP `StdioServerParameters`
  construction out into `require_api_key()`/`mcp_server_params()` so both entry points
  share them. CLI behavior unchanged (still prints the same `[tool call]`/`[tool result]`
  lines via a default callback) -- re-tested after the refactor to confirm no regression.
- New `go2_llm_nav/web_ui.py` (new console script `go2_llm_nav_web`, `aiohttp`-based,
  already available in the environment -- no new system dependency needed): a single
  self-contained HTML page (dark chat UI, sidebar of known places as clickable buttons,
  a live status badge polling `get_navigation_status` every 3s) served locally, backed
  by the same persistent MCP session + `ChatClient` the CLI uses. `/api/chat` runs
  `run_agent` and returns the reply plus the full tool-call/result transcript for the
  frontend to render; `/api/places`, `/api/status`, `/api/info` are small direct
  MCP-tool-call passthroughs for the sidebar/status badge (bypass the LLM, so those
  don't cost API quota). `list_places`' tool result is multiple concatenated pretty-
  printed JSON objects (not a JSON array) -- added `_split_json_objects` (brace-depth
  scanning) to parse it back into a list for the sidebar.
- `run_go2_demo.sh`/`run_go2_demo_junior.sh`: the final hand-off now launches
  `go2_llm_nav_web` by default (opens the browser automatically) instead of the terminal
  REPL; `GO2_UI=cli` reverts to the old terminal chat.
- Verified live end-to-end via direct HTTP calls (not just "should work"): `/`, `/api/
  info`, `/api/places`, `/api/status` all correct; `/api/chat` with "what places are
  there?" (no navigation) and "go to the sofa"/"go to the table" (real navigation)
  confirmed via `/odom` that the robot actually moved; "cancel" correctly called
  `cancel_navigation` and flipped status to `aborted`.
- **Found a real, unrelated Nav2 issue while doing this verification:** navigating from
  near the sofa to the table got stuck in a recovery loop -- `number_of_recoveries`
  climbed to 20+ over 20s with the robot barely moving and `distance_remaining_m` stuck
  at `0.0` the whole time (a stale/wrong distance reading during the loop, not real
  proximity to the goal). Not investigated further this session (out of scope for "build
  a GUI"), cancelled cleanly via the GUI's own cancel button/command to leave a clean
  state. Worth a dedicated look later -- may be specific to that particular start/goal
  pair (near-sofa -> table) rather than a general regression, since sofa/table goals from
  the origin have both worked cleanly in numerous earlier tests this session.
- **Process-cleanup mistake caught and fixed during this session's testing:** an earlier
  "close everything" cleanup silently failed because one `pkill` in an unguarded chain
  returned nonzero (nothing matched) and aborted the rest of the chain before it reached
  `nav2_container`/`junior_ctrl` -- exactly the documented harness gotcha, self-inflicted
  by not adding `|| true` to every line. Left `junior_ctrl`/`nav2_container` running for
  ~35 minutes while believing they'd been stopped; briefly misread them as the user's own
  independently-started session before working out they were mine. Killed by explicit
  PID once identified. No user-visible harm (their own terminal commands happened to
  reach the still-alive Nav2 stack successfully), but a good reminder to actually verify
  a cleanup's `ps` output rather than trusting a silent/aborted chain.

### 2026-08-23 — Session 10 (cont'd): switched the LLM from local Ollama to Gemini
- User asked to switch off the local `llama3.2:3b`/Ollama model to Gemini (this had been
  offered as an option in an earlier, still-open thread about tool-calling reliability).
- Rewrote `go2_llm_nav/agent.py`'s `OllamaClient` into a generic `ChatClient` that talks
  to any OpenAI-compatible `/chat/completions` endpoint (Gemini, Ollama, Groq, ... --
  whichever is just a matter of `GO2_LLM_BASE_URL`/`GO2_LLM_API_KEY`/`GO2_LLM_MODEL` env
  vars, no code change needed to switch providers again later). Default base URL is
  Gemini's OpenAI-compat endpoint (`https://generativelanguage.googleapis.com/v1beta/openai`).
  Fixed a latent crash while at it: `(message.get("content") or "").strip()` -- the old
  code crashed on a `None` content field, which OpenAI-style tool-call-only turns
  legitimately have.
- User's first pasted "API key" was actually a Google OAuth sign-in code (`AQ.` prefix),
  not an API key -- caught by format mismatch before wasting a test. Their real key
  (from `aistudio.google.com/app/apikey`) also happened to start with `AQ.`, which
  contradicted my assumption that all Gemini keys start with `AIza` -- tested it directly
  against the API rather than continuing to argue from a memorized format, and it worked.
  Worth remembering: don't gate correctness on a remembered format when you can just test
  the real endpoint.
- Verified live end-to-end with the real key: basic connectivity, multi-turn tool-calling
  (list_places -> navigate_to_place chaining), a real navigation goal that actually moved
  the robot (confirmed via `/odom`, both on `go2_simulation` and, separately, the real
  `junior_ctrl` walking robot -- confirmed via `/joint_states` showing genuine leg
  articulation, not just launch success), correct honest handling of an invalid place
  name (declined, listed real options, no hallucination), and correct status reporting.
  Across all tests, zero repeats of either of the two local-model failure modes documented
  in Session 5/9/10 (hallucinated tool call, misreported failure) -- Gemini has been
  reliable so far, though this is not yet a large enough sample to call it proven.
- **Hit and fixed a real reliability issue of Gemini's own:** a `503 Service Unavailable`
  (transient server-side) and later a `429` free-tier rate limit both occurred live during
  testing. Added retry-with-backoff (honoring `Retry-After` when present) for `429`/`5xx`
  in `ChatClient.chat()`, and wrapped `run_agent`'s LLM call in a try/except so a
  persistent failure degrades to a clear one-line message in the chat instead of crashing
  the whole `ros2 run` process with a raw Python traceback -- this exact crash happened to
  the user live before the fix.
- **Model default changed twice during testing, for a real reason:** started with
  `gemini-3.7-flash` (the newest/most capable flash model) but its free tier turned out to
  be capped at just 20 requests/day (`GenerateRequestsPerDayPerProjectPerModel-FreeTier`
  quota, confirmed via the API's own error body) -- burned through by testing alone within
  the session. Tried `gemini-3.6-flash` next: worked, but its responses carry large hidden
  "thinking" token overhead (664 total tokens for a 3-word reply to an 8-token prompt),
  which risks the same problem under token-based quotas. Settled on
  `gemini-3.1-flash-lite`: efficient (15 total tokens for the same test), and verified
  reliable across every tool-calling scenario tested above -- now the default in both
  `agent.py` and both demo scripts.
- Removed all Ollama-specific logic from `run_go2_demo.sh` and `run_go2_demo_junior.sh`
  (starting/waiting-for `ollama serve`, model pull-if-missing) and replaced it with a
  `GEMINI_API_KEY`/`GO2_LLM_API_KEY` presence check that fails fast with a clear message
  and a link to `aistudio.google.com/app/apikey` if neither is set.
- Rewrote the relevant parts of `RUN_DEMO.md`: new "One-time setup: a free Gemini API
  key" section (with the 20-req/day quota gotcha called out explicitly so it isn't
  rediscovered the hard way again), "Known rough edges" reworked to cover Gemini's 429s
  as the live concern and demote the old local-model issues to historical context, manual
  step-by-step Terminal 4 section stopped referencing Ollama, and removed a stale
  "real-walking robot not covered by this guide" note left over from before
  `run_go2_demo_junior.sh` existed.

### 2026-08-23 — Session 10 (cont'd): real walking robot wired into the visual demo too
- User closed the GUI/RViz windows from the earlier fix, noticed the robot's legs never
  moved, and asked why -- correctly pointing out they'd originally provided a package
  that controls the Go2 normally (real legs). Answer: `run_go2_demo.sh` only ever wired
  up `go2_simulation`, a deliberately simplified robot with every leg joint set to
  `type="fixed"` in its URDF (confirmed by grepping the URDF -- all 12 leg joints plus
  every other joint are fixed) -- it slides as a rigid block, by design, to test Nav2/
  SLAM/LLM without depending on a walking controller's stability. The user's real
  controller (`junior_ctrl`/`unitree_guide2`, revolute leg joints, confirmed via the
  other workspace's `leg.xacro`) was never wired into the visual one-command demo.
- Built `run_go2_demo_junior.sh`, a parallel one-command launcher for the real robot:
  sources both workspaces, launches `unitree_go2_sim`'s `unitree_go2_nav_bringup.launch.py`
  (auto-stands the robot via `auto_stand.py`'s FSM automation, waits for its "now
  listening to /cmd_vel" log line rather than a fixed sleep), then `go2_navigation`'s
  `nav2_junior.launch.py`, reads the live initial pose from `/odom` (same fix as Session
  10's first entry), starts Ollama, sets `GO2_PLACES_FILE` to `places_junior.yaml`, and
  hands off to the same `go2_llm_nav` chat REPL.
- To make Gazebo/RViz actually visible for this robot too:
  - `unitree_go2_nav_bringup.launch.py` was passing `gui: false` to the underlying sim
    launch -- flipped to `true`. Left `rviz: false` there deliberately (its own bundled
    `rviz.rviz` has no map/AMCL/costmap displays) and instead...
  - ...added an RViz2 node to `go2_navigation/launch/nav2_junior.launch.py`, reusing the
    same `go2_nav_view.rviz` config built for the other demo (generic, topic-name based,
    works for either robot), gated by a `use_rviz` arg exactly like `nav2.launch.py`.
- **Found and fixed a real bug in `run_go2_demo.sh`'s cleanup while testing this:** the
  `pkill -9 -f "ign gazebo.*go2_world.sdf"` pattern never actually matched the real,
  long-running `ign gazebo server`/`ign gazebo gui` processes -- checked their actual
  argv (`/proc/<pid>/cmdline`) and confirmed those child processes carry neither the
  world path nor any launch-script name, only the short-lived wrapper process that
  spawns them does. This meant Gazebo was silently surviving the script's own cleanup
  and being left running after `quit`/Ctrl-C this whole time. Fixed in both scripts:
  broadened to `pkill -9 -f "ign gazebo"` (also added `rviz2` to the kill list, missing
  before).
- Verified live end-to-end, not just "should work": ran `run_go2_demo_junior.sh` for
  real (via the same nohup/EOF harness limitation noted below -- confirmed it reaches
  "Everything is up", prints the REPL prompt, then cleans up completely and correctly on
  EOF). Separately, drove each stage manually to check the actually-interesting parts:
  Gazebo GUI + RViz both really open (checked via `ps`, not just launch success), AMCL
  localizes (non-identity `map->odom` TF), and a real LLM prompt ("navigate to the
  waypoint") produced a `navigate_to_place` call that Nav2 accepted, walked the robot
  from (0.13, 0.05) to (0.74, 0.45), with `/joint_states` showing real non-zero hip/
  thigh/calf angles throughout (not the locked-at-zero values `go2_simulation` would
  show) -- confirmed via telemetry, not the chat transcript, per the standing "verify via
  telemetry, don't trust a status string" practice.
- Also cleaned up incidental clutter found while testing: five separate stray `ollama
  serve`/`llama-server` processes had accumulated over the session's testing (never
  killed between manual test runs, ~2.8GB RAM each) and were very likely why some
  `go2_llm_nav` calls were mysteriously hanging/timing out earlier in this session --
  killed all but one. Worth checking `ps aux | grep llama-server` if `go2_llm_nav` ever
  seems to hang for no reason.
- `RUN_DEMO.md` rewritten with a new "Two demos -- which one to run" section up top
  explaining the sliding-vs-walking distinction plainly, so this doesn't need
  rediscovering next time.

### 2026-08-23 — Session 10: real-usage bug report fixed (stale initial pose + misreported failure)
- User ran `run_go2_demo.sh` for real (not a fresh sim -- one was already running from
  earlier testing, robot had moved from the origin), asked "go to the entrance", and hit
  a real failure: `navigate_to_place` returned `{"ok": false, "error": "navigate_to_pose
  rejected the goal."}`, but the LLM's reply was "Navigation has started. Please wait..."
  -- two bugs in one transcript.
- **Bug 1 -- stale initial pose.** `run_go2_demo.sh` always published `(x=0, y=0, z=0)` as
  the initial pose whenever it started Nav2 fresh, on the assumption the robot spawns at
  the origin. True for a truly fresh sim launch; false here, since the sim was reused from
  an earlier session and the robot had since moved. AMCL was told a false starting
  position, so Nav2 rejected every subsequent goal. Fixed by reading the robot's actual
  live position from `/odom` (embedded `rclpy` one-shot subscriber, x/y/qz/qw) instead of
  hardcoding zeros. Verified by reproducing the exact scenario: drove the robot ~4.5m from
  the origin via `/cmd_vel`, reused the sim, ran the fixed script -- it logged "Setting
  initial pose to (x=4.52..., y=5.1e-07) -- read live from /odom," and the same
  `navigate_to_place({'name': 'entrance'})` call that had failed before now returned
  `{"ok": true, ...}`.
- **Bug 2 -- misreported failure.** Separately, even on a call that *did* correctly
  execute and correctly return `ok: false`, the LLM's summary ignored that and reported
  success anyway -- a third distinct LLM-reliability failure mode (Session 5/9 documented
  hallucinated tool calls; this is the model calling the tool correctly but misreading its
  own result). Fixed by tightening `go2_llm_nav/agent.py`'s `SYSTEM_PROMPT` to explicitly
  require checking the `ok`/`error` fields and reporting failures honestly, and to not
  call `get_navigation_status`/`cancel_navigation` proactively after a successful start.
  Rebuilt `go2_llm_nav`. Caveat: the re-test after this fix happened to hit a success case
  (Bug 1 was also fixed by then), so the "reports failure honestly" behavior itself has
  **not yet been independently proven** against a real `ok: false` result -- next time one
  occurs, check that the LLM's reply actually says it failed.
- Updated `RUN_DEMO.md`'s "Known rough edge" section (now "rough edges", plural) with both
  fixes, and added a caveat to the manual step-by-step Terminal 3 instructions about
  checking `/odom` before assuming `(0,0,0)` when reusing an existing sim.
- Still open, unrelated to this bug report: whether to switch `go2_llm_nav` off the local
  `llama3.2:3b` model to a free online API (Groq/Llama 3.3 70B recommended, or Gemini) for
  better tool-calling reliability -- offered to the user, no decision made yet.

- **Also this session: made the simulation and localization visible.** User asked for
  Gazebo and RViz to actually show up on screen -- the demo had been running Gazebo
  headless (`-s` flag, server-only) with a forced-software-rendering workaround
  (`LIBGL_ALWAYS_SOFTWARE`/`MESA_GL_VERSION_OVERRIDE`) left over from before the Session 8
  GPU driver fix, and there was no RViz launch at all. Confirmed the GPU driver is still
  healthy (`nvidia-smi` shows the GTX 1650 Ti fine, `glxinfo -B` shows hardware direct
  rendering via the Intel iGPU too) before changing anything.
  - `go2_simulation/launch/sim.launch.py`: removed `-s` from `gz_args` (was headless) and
    removed the two forced-software-rendering env vars (no longer needed).
  - `go2_navigation/launch/nav2.launch.py`: added an RViz2 node (new `go2_navigation/rviz/
    go2_nav_view.rviz`, based on `nav2_bringup`'s default view with `RobotModel` display
    turned on) gated by a new `use_rviz` launch arg (default `true`). Launched as a plain
    `Node`, not `nav2_bringup`'s own `rviz_launch.py`, deliberately -- that file emits a
    `Shutdown` event when RViz exits, which would take Nav2 down with it; a plain node
    just closes its own window.
  - Rebuilt both packages, then verified live (not just "should work"): launched sim +
    Nav2 directly, confirmed via `ps` that `ign gazebo gui` and `rviz2` (with the new
    config) were both actually running, confirmed AMCL localized (non-identity `map->odom`
    TF, converged `/amcl_pose` covariance), then ran `go2_llm_nav` with a single prompt
    and confirmed via live `/odom` that the robot actually walked from the origin to
    ~(1.67, 1.63) -- next to the sofa -- and `get_navigation_status` returned `idle`
    (arrived) afterward.
  - Incidentally reproduced the known LLM-hallucination bug twice in a row during this
    same verification ("go to the sofa" got only a `list_places` call, no
    `navigate_to_place`, yet a confident "navigation has started" reply) before a more
    explicit prompt triggered the real tool call -- consistent with, not a regression of,
    the existing documented limitation.
  - Also found and cleaned up an unrelated stale process during this session: a
    `slam_junior.launch.py` instance left running since 2026-08-21 (Milestone 9's junior
    map redo, already completed) was still alive and could collide topic/TF names with a
    fresh `go2_simulation` launch -- killed it. Separately noted for future sessions: the
    `ros2` CLI daemon's graph cache can go stale after repeated hard `kill -9`s in a short
    span (ghost/duplicate node names, topics that appear published when nothing is
    actually publishing) -- `ros2 daemon stop` (it auto-restarts on the next command)
    clears it.

### 2026-08-23 — Session 9: one-command demo + user-facing run guide
- Direction from user: confirm the full pipeline is actually ready for "go to the sofa"
  end-to-end use, then build a nice interface and a launch guide.
- Confirmed: yes, for `go2_simulation`'s test room (which has real `sofa`/`table`/
  `entrance` places) -- this was already proven working in Session 5/8.
- Built `run_go2_demo.sh`: one command starts the simulation, Nav2 (with the saved map),
  and Ollama in the right order, waits for each to actually be ready (not just
  "launched"), sets the initial pose automatically (the robot always spawns at the
  origin), then hands off to the LLM chat REPL. Ctrl-C or `quit` cleans up everything it
  started. Idempotent -- safe to re-run if the simulation's already up.
- Found and fixed two real bugs in the script itself while testing it end-to-end (not
  left for the user to hit): `set -u` breaks on sourcing ROS's own `setup.bash`
  (references unbound variables internally) -- removed it; ending the script with `exec`
  for the chat REPL would have silently skipped the cleanup trap on normal exit -- removed
  the `exec`. Also caught the script defaulting Ollama to the wrong model directory
  (forgot to export `OLLAMA_MODELS`), which would have wastefully re-downloaded a model
  that was already present -- fixed and verified it reuses the existing one.
- Wrote `RUN_DEMO.md` as a separate, user-facing "how do I run this" doc (kept deliberately
  short, distinct from this file's full technical history) -- includes the manual
  step-by-step version too, for anyone who wants to understand/control each piece.
- While testing the finished script twice in a row, reproduced the known LLM-hallucination
  failure mode again (documented in Session 5): "go to the sofa" worked correctly (real
  `navigate_to_place` tool call, confirmed by the `[tool call]` log line), but "go to the
  table" immediately after did not -- the model wrote `navigate_to_place("table")` as
  plain reply text without ever actually invoking the tool. Documented this plainly in
  `RUN_DEMO.md` rather than hiding it, including how the user can tell the difference
  themselves in the transcript, and offered `GO2_LLM_MODEL=llama3.1:8b` as a
  larger-but-slower alternative.

### 2026-08-21 — Session 8: GPU driver fix resolved the walking instability
- User pushed back on Session 7's "balance controller needs tuning" conclusion (had run
  `junior_ctrl` smoothly elsewhere before) and asked to properly verify the GPU driver
  rather than accept it. Correct call: found `nvidia-smi` failing with a genuine, reboot-
  fixable driver/kernel-module version mismatch (package upgraded after last boot, DKMS
  had the matching module built but not loaded).
- Measured real-time factor directly (wall-clock vs. `/clock`) before asking for a reboot:
  ~0.76 under forced CPU software rendering. Saved this whole finding + a concrete
  re-test plan to memory before the user rebooted, so the next session could pick it up
  without re-deriving anything.
- Post-reboot: confirmed `nvidia-smi` works, hardware OpenGL rendering works, re-measured
  RTF without the software-rendering workaround: ~0.997 (essentially real-time).
- Redid every walking test that had previously fallen (Milestone 8's keyboard trot,
  Milestone 9's sustained 0.2 m/s straight walk, the full multi-leg exploration sequence,
  in-place rotation) back to back at the same commanded speeds -- all passed cleanly and
  repeatably this time, confirmed via live `/odom` orientation/position each time, not
  assumed from the improved RTF number alone.
- Updated Milestone 8/9 status to reflect the real root cause (simulation timing, not
  controller tuning), then immediately did the flagged follow-ups rather than leaving them
  open: tested a higher speed (0.35 m/s, stayed upright) and raised
  `nav2_params_junior.yaml`'s velocity limits to that tested value; redid the SLAM mapping
  pass with a real translation-heavy exploration (previously only safe to do rotation-only)
  -- `junior_world_map` now resolves actual obstacle features (both cylinders, two box
  corners) instead of the old single-vantage sparse pattern.
- During that longer exploration session, **one fall did occur** (same safety-trip
  signature as before). Calibrated the conclusion accordingly in `PROGRESS.md`: the GPU fix
  is a real, large improvement (every short targeted retest of a previously-100%-failing
  maneuver now passes reliably), not a provable "never fails again" -- reported as such
  rather than overclaiming a full fix.

### 2026-08-21 — Session 7: wired real walking into Nav2/SLAM/LLM
- Direction from user: wire the Milestone 8 real-walking package into Nav2, SLAM, and the
  LLM layer.
- Found `junior_ctrl` already has a purpose-built `State_move_base` FSM state that
  subscribes to `/cmd_vel` -- didn't need to build Nav2 integration into it, just needed
  to trigger it (key `5` from FixedStand).
- Wrote `auto_stand.py` (pty-based) to fully automate Passive→FixedStand→MOVE_BASE, since
  `junior_ctrl` only accepts state commands via live keyboard input, no ROS interface for
  it. Combined into one new launch file with the sim and a `pointcloud_to_laserscan` node.
- Found and fixed two more real bugs at the source (not worked around): `odom->base_link`
  was silently never published on TF at all (only the `/odom` topic worked) because the
  vendored package's odometry plugin config was missing a `tf_topic` element; and the same
  `robot_description`-as-YAML launch crash seen before, fixed the same way
  (`ParameterValue(..., value_type=str)`).
- Added junior_ctrl-specific SLAM/Nav2 config to `go2_navigation` (`base_link` instead of
  `base_footprint`, conservative velocity limits) and a junior-specific semantic map to
  `go2_semantic_map`; added `GO2_PLACES_FILE` env var support to `go2_mcp_server` so the
  existing MCP/LLM code works against either robot with zero code changes.
- Hit the Milestone 8 balance instability directly and repeatedly while trying to map the
  world: two separate full/gentle exploration attempts both fell (once even at 0.15 m/s
  straight-line) while in-place rotation never once failed across many repeats -- so the
  saved map only covers what a single rotating vantage point can see, not the world's
  actual (4-7m away) landmarks. Defined semantic places only at real, tested, nearby
  points rather than the unreachable landmark coordinates.
- **Proved the wiring works**: one full autonomous `NavigateToPose` run completed
  (`SUCCEEDED`) with the robot staying upright the entire ~20s, confirmed via live
  position/orientation reads, not just trusting the status string (a lesson from earlier
  sessions).
- Also caught the LLM (`llama3.2:3b`) hallucinating a tool call it never made ("navigation
  has started" with no actual `navigate_to_place` call, confirmed by the robot not having
  moved) -- called the same tool directly afterward to confirm the underlying logic still
  works correctly; the model's reasoning reliability, not the plumbing, was at fault.
- Net result: wiring done and proven; walking stability remains the real, pre-existing,
  documented limiting factor for anything beyond short/careful autonomous runs.

### 2026-08-21 — Session 6: real walking, via a user-provided package
- User pointed out the Go2 in Milestones 1–7 only slides (legs locked); asked if an
  existing controller could give real walking, suspected something for Ubuntu 24.
  Researched CHAMP as one real option (see prior session's findings), then the user
  directly provided a different, more complete package
  (`unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy`, a `unitree_guide2`/`junior_ctrl`
  FSM+balance controller, not CHAMP) and asked to build and test it as-is.
- Diagnosed and fixed a pre-existing system package conflict (classic Gazebo 11 vs. the
  modern `gz-tools2` suite) blocking apt installs; user ran the `sudo` installs (this
  session has no passwordless sudo).
- Built all 5 sub-packages cleanly. Found and permanently fixed a real bug: the launch
  never actually started `controller_manager` (silently, no error anywhere, even at `-v
  4`) because `ros-humble-gz-ros2-control` doesn't add itself to gz-sim's plugin search
  path — patched both launch files to set `GZ_SIM_SYSTEM_PLUGIN_PATH` explicitly, verified
  fixed with a from-scratch launch.
- Along the way, found and cleaned up several of my own orphaned processes (from earlier
  debugging attempts where I'd only killed the top-level `gz sim` process, not its
  children) plus the old unrelated Nav2 stack still running from Session 1–5 — together
  they'd pushed the machine to a load average of 24, which was independently causing
  `controller_manager`'s service calls to hang even after the real bug was fixed.
- Verified real walking end to end via `junior_ctrl` (only controllable via a live
  keyboard TTY, no ROS topic interface — used `screen` to give it one): commanded
  `FixedStand`, watched it genuinely stand upright (`tilt=1.000`) over ~14s; commanded
  `Trotting` + forward velocity, watched joint angles start cycling asymmetrically (real
  gait) and `/odom` move >1 m. Also caught it falling over mid-trot on flatter testing
  (safety monitor correctly forced it back to passive) — reported honestly as "working,
  not yet stable," not overstated as a finished replacement for the Nav2-ready sim.
- This package is a separate, unintegrated addition — nothing in Milestones 1–7 changed,
  and this isn't wired to Nav2/SLAM/the semantic map/LLM layer yet.

### 2026-08-21 — Session 5: LLM layer, full pipeline live
- Direction from user: integrate an LLM, free if possible, to actually test whether the
  framework works end to end.
- Installed Ollama locally (no sudo/GPU on this machine — worked around both; see
  Milestone 7). Pulled `llama3.2:3b` after a larger model's download failed on flaky
  network + was going to take too long.
- Built `go2_llm_nav`: an MCP-client + Ollama agentic-loop front end. Fixed two small bugs
  (`input_schema` field name, missing `httpx`) and one real one (the model
  self-cancelling a goal it got impatient waiting on — fixed via system prompt, not code).
- **Verified the full pipeline live, twice**, including a real ~3.5 m cross-room
  LLM-triggered trip with a correctly-handled mid-conversation status check. This is the
  first time the complete `prompt → LLM → MCP → Nav2 → real robot motion` chain has been
  demonstrated working end to end.
- Left `ollama serve` running locally (like the sim/Nav2 stack) for further testing.

### 2026-08-20 — Session 4: semantic map + MCP server
- Direction from user: don't chase the Nav2 controller/physics issue further right now
  (noted a possible future option: a real quadruped controller package, e.g. `champ`,
  instead of the current kinematic hack); build the semantic map manually; start the
  API/LLM integration so a person's prompt can be turned into robot motion.
- Built `go2_semantic_map`: hand-authored `places.yaml` (name → approach pose + yaw +
  description) and a small loader. 3 places defined for the test world (`sofa`, `table`,
  `entrance`).
- Built `go2_mcp_server`: a real MCP server (installed the official `mcp` Python SDK via
  pip) exposing `list_places`, `navigate_to_place`, `get_navigation_status`,
  `cancel_navigation` over stdio, backed by a lock-protected, on-demand-spun rclpy
  `NavigateToPose` action-client wrapper (no background spin thread).
- Verified all 4 tools twice: once calling the tool logic directly against the live
  sim+Nav2 stack (including a real ~5 m cross-room trip and a mid-transit cancel that was
  confirmed by checking `/odom` twist dropped to zero, not just trusting the return
  value), and once through an actual `mcp.ClientSession` over real stdio JSON-RPC talking
  to the installed executable.
- Hit and fixed a build issue: `colcon build --symlink-install` fails on these two new
  `ament_python` packages (`--editable` not recognized by this setuptools version) — use
  plain `colcon build` for them instead.
- Remaining gap before "person types a prompt and the robot goes": the LLM layer
  (`go2_llm_nav`) that actually calls these tools from free text isn't built yet — this
  MCP server is the ready-to-use integration point for it.

### 2026-08-20 — Session 3: scope check-in
- Direction from user: Nav2 goal-arrival precision (Milestone 3's known 0.46 m gap) is
  **not a priority right now** — leaving it as-is rather than continuing to tune.
- Clarified and documented URDF provenance in detail (Milestone 1): confirmed the robot
  geometry/links/joints/masses/meshes come unmodified from a *community* ("unofficial")
  Go2 project, not fetched fresh from Unitree's own repo — though that community file's
  own header shows it originates from Unitree's real CAD export, so the geometry itself is
  faithful. Explicitly called out the one real structural change made (12 leg joints
  `revolute` → `fixed`) versus pure additions (LiDAR frame/sensor, IMU sensor tag, Gazebo
  plugins) and a removal of non-official cruft (fake `map`/`odom` links from the source
  project).

### 2026-08-20 — Session 2: AMCL tuning, full map, physics investigation
- Tuned AMCL alphas/update thresholds (see Milestone 3 tuning history); net improvement in
  goal-arrival accuracy (0.69 m → 0.46 m) but not fully within tolerance yet.
- Redid full-room exploration with a safer pattern; saved a clean, correctly-proportioned
  map covering the whole room and both obstacle types.
- Found and diagnosed the robot-tipping bug; tried and rejected a gravity-on fix (works for
  isolated bumps, fails for ordinary turning); reverted to the known-stable gravity-off
  config; documented the correct fix (planar virtual-joint chain) as the top follow-up item.
- Caught and fixed a harness/tooling issue on my end: `nohup cmd & disown` inside a
  `run_in_background` Bash call defeats the tool's own completion tracking, causing false
  "task complete" notifications mid-navigation. Switched to letting the tool background the
  blocking command directly.

### 2026-08-20 — Session 1: Milestones 1–3 first pass
- Surveyed the existing `unitree_ros2` folder (real-robot DDS bridge only, no sim) and
  found a separate, more complete real-robot project (`~/ros2_ws/src/go2_robot_sdk`) on the
  same machine; reused its Go2 meshes/URDF as a visual base after stripping out its faked
  static `map/odom` frames.
- Confirmed this machine runs gz-sim/Ignition Fortress (not classic `gazebo_ros`), and that
  its GPU driver is broken (NVIDIA userspace/kernel mismatch) — worked around with headless
  server mode + forced Mesa software rendering.
- Built `go2_simulation` (Milestone 1) and `go2_navigation` (Milestones 2–3) from scratch;
  found and fixed a LiDAR self-hit bug; got slam_toolbox mapping and a first Nav2 goal
  (`SUCCEEDED`, but off-target — the discrepancy that motivated Session 2's AMCL work).
