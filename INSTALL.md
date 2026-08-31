# Installing Go2 Natural-Language Navigation

Written for a clean **Ubuntu 22.04** machine. Everything below has to succeed once;
after that, `./run_go2_demo.sh` or `./run_go2_demo_junior.sh` is all you ever need
(see [USAGE.md](USAGE.md)).

## 1. ROS 2 Humble + Gazebo (gz-sim / Ignition Fortress)

If you don't already have ROS 2 Humble installed, follow the official instructions:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

This project uses `gz-sim` (Ignition Fortress), not classic `gazebo_ros` — install the
ROS <-> Gazebo bridge packages:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-pointcloud-to-laserscan ros-humble-gz-ros2-control
```

## 2. NVIDIA GPU driver (recommended)

Object detection (YOLO-World) and Gazebo's sensor rendering both want a working NVIDIA
GPU. Confirm the driver is actually healthy before going further:

```bash
nvidia-smi   # should print your GPU, driver version, no errors
```

If this fails or shows a driver/kernel-module mismatch, fix that first (often just a
reboot after a driver update) — a broken driver silently degrades simulation real-time
performance in ways that look like unrelated robot instability. See `PROGRESS.md`
("GPU-driver/RTF") if you hit this.

**Laptop/hybrid-graphics machines**: if the GPU is an NVIDIA card alongside integrated
graphics (Optimus/PRIME), Gazebo's *offscreen* sensor rendering (used for the camera,
separate from the visible GUI window) can fail to get a GPU context and silently
produce blank camera frames. Both demo scripts already set the required environment
variables before launching (`__EGL_VENDOR_LIBRARY_FILENAMES`,
`__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`) — nothing extra to
do here, just know this is why those variables are there if you're reading the scripts.

## 3. Python dependencies

```bash
# LLM agent + MCP server + desktop/web chat UI
pip install --user mcp httpx aiohttp pywebview

# Object detection (YOLO-World via yolo_ros) -- only needed for run_go2_demo_junior.sh
pip install --user torch --index-url https://download.pytorch.org/whl/cu121  # CUDA build; use the plain `pip install torch` if you have no GPU
pip install --user ultralytics lap
pip install --user git+https://github.com/ultralytics/CLIP.git
```

The CLIP install step matters: `ultralytics`' own auto-install for it has been observed
to silently fail — if you skip this and later see `No module named 'clip'` when running
the junior demo, this is the fix.

**Model weight files** (`yolov8s-worldv2.pt`, `weights/clip/ViT-B-32.pt`) aren't
committed to this repo — they're large (the CLIP one alone is 338MB, over GitHub's
100MB file limit) and downloadable, not source. `ultralytics`/CLIP fetch them
automatically the first time they're actually needed (confirmed live: the first call
that needs the CLIP weights triggers an on-demand ~338MB download, roughly 20-25s on a
good connection) — expect the *first* real detection run to take noticeably longer
than every run after it, not a hang.

## 4. Get a free Gemini API key

The LLM is Google's Gemini — free, no credit card required:

1. Go to **https://aistudio.google.com/app/apikey** and click **"Create API key."**
2. Add it to your shell so every future run picks it up automatically:
   ```bash
   echo 'export GEMINI_API_KEY="your-key-here"' >> ~/.bashrc
   source ~/.bashrc
   ```

(Any other OpenAI-compatible endpoint works too, including a local model server like
Ollama — see `GO2_LLM_BASE_URL` in [USAGE.md](USAGE.md).)

## 5. Build both workspaces

This project is two separate colcon workspaces (kept independent on purpose — see
`README.md`). Build each with its own ROS environment sourced first:

```bash
# Workspace 1: the main pipeline (LLM, MCP server, Nav2 config, both simulated worlds)
cd /home/kan/lab/course
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src

# Workspace 2: the real-walking robot + object detection
cd /home/kan/lab/course/unitree_go2_ros2_jazzy1/unitree_go2_ros2_jazzy
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
cd /home/kan/lab/course/unitree_go2_ros2_jazzy1
colcon build
```

`rosdep install` pulls in the ROS-side dependencies declared in each package's
`package.xml` (Nav2, `slam_toolbox`, `ros_gz_*`, etc.) — the apt packages from step 1
cover the common ones already, so this step mainly catches anything missed.

**`--base-paths src` matters** for workspace 1: `unitree_go2_ros2_jazzy1/` and
`unitree_ros2/` live alongside `src/` at the project root as their own independent
workspaces (see README.md) — a bare `colcon build` with no `--base-paths` recursively
discovers packages anywhere under the current directory, so it would also try (and
fail) to build their packages as part of this workspace. `--base-paths src` scopes
discovery to just this workspace's own 5 packages.

## 6. Verify

```bash
cd /home/kan/lab/course
./run_go2_demo.sh
```

This should: launch Gazebo, bring up Nav2, confirm your Gemini key, then open a chat
window. Type `"go to the sofa"` — if the robot walks there, everything is installed
correctly. Ctrl-C in the terminal shuts everything down cleanly.

For the real-walking robot with object detection: `./run_go2_demo_junior.sh` (takes
longer — the robot has to physically stand up first, and YOLO-World needs a few extra
seconds to load its model).

If either script prints a clear error (missing API key, a `ros2 launch` failure, etc.),
it's telling you exactly what's missing — fix that and re-run. For anything not covered
here, see [USAGE.md](USAGE.md)'s "known rough edges" or `PROGRESS.md`'s full bug
history (many issues below the surface — camera rendering, initial-pose quaternions,
turn-speed limits — have already been hit and fixed once, with the reasoning recorded
there).
