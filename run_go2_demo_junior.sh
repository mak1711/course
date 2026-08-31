#!/bin/bash
# One-command launcher for the REAL-walking-robot version of the natural-language
# navigation demo -- same idea as run_go2_demo.sh, but for junior_ctrl
# (unitree_guide2), which has actual revolute leg joints and a real gait/balance
# controller, instead of go2_simulation's simplified locked-leg "sliding" robot.
#
# Brings up: the junior_ctrl Gazebo simulation (auto-stood into MOVE_BASE so it
# accepts /cmd_vel), Nav2 (localized against the saved junior map), and finally drops
# you into the same interactive chat prompt as the other demo. Talks to Gemini for the
# LLM (needs a free API key -- see USAGE.md); no local model server to start.
#
# Usage: ./run_go2_demo_junior.sh
# Stop:  Ctrl-C (cleans up everything it started)

# Note: no `set -u` -- ROS 2's own setup.bash references unbound variables internally,
# so nounset mode breaks sourcing it. -o pipefail alone is safe here.
set -o pipefail

COURSE_DIR="/home/kan/lab/course"
JUNIOR_WS="/home/kan/lab/course/unitree_go2_ros2_jazzy1"
LOG_DIR="/tmp/go2_demo_junior_logs"
mkdir -p "$LOG_DIR"

GREEN='\033[0;32m'; YELLOW='\033[0;33m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
info()  { echo -e "${GREEN}[go2-junior-demo]${NC} $1"; }
warn()  { echo -e "${YELLOW}[go2-junior-demo]${NC} $1"; }
err()   { echo -e "${RED}[go2-junior-demo]${NC} $1"; }

CHILD_PIDS=()
cleanup() {
    echo
    info "Shutting down everything this script started..."
    for pid in "${CHILD_PIDS[@]:-}"; do
        kill -9 "$pid" >/dev/null 2>&1 || true
    done
    # ros2 launch spawns its own process tree; the actual long-running "ign gazebo
    # server"/"ign gazebo gui" processes don't carry any launch-script name or world
    # path in their argv, so match on "ign gazebo" alone or they survive cleanup.
    pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
    pkill -9 -f "auto_stand.py" >/dev/null 2>&1 || true
    pkill -9 -f "junior_ctrl" >/dev/null 2>&1 || true
    pkill -9 -f "nav2_container" >/dev/null 2>&1 || true
    pkill -9 -f "rviz2" >/dev/null 2>&1 || true
    pkill -9 -f "yolo_node\|tracking_node\|debug_node\|detect_3d_node" >/dev/null 2>&1 || true
    info "Done."
}
trap cleanup EXIT INT TERM

wait_for_ros_topic() {
    local topic="$1" timeout_s="$2" waited=0
    while ! ros2 topic list 2>/dev/null | grep -qx "$topic"; do
        sleep 1; waited=$((waited + 1))
        if [ "$waited" -ge "$timeout_s" ]; then
            err "Timed out waiting for $topic after ${timeout_s}s -- check $LOG_DIR for logs."
            return 1
        fi
    done
    return 0
}

wait_for_log_pattern() {
    local logfile="$1" pattern="$2" timeout_s="$3" waited=0
    while ! grep -qE "$pattern" "$logfile" 2>/dev/null; do
        sleep 1; waited=$((waited + 1))
        if [ "$waited" -ge "$timeout_s" ]; then
            err "Timed out waiting for '$pattern' in $(basename "$logfile") after ${timeout_s}s."
            return 1
        fi
    done
    return 0
}

echo -e "${BOLD}=== Go2 natural-language navigation demo (REAL walking robot) ===${NC}"
echo

source /opt/ros/humble/setup.bash
source "$JUNIOR_WS/install/setup.bash"
source "$COURSE_DIR/install/setup.bash"

# The camera's *offscreen* sensor rendering (separate from the interactive Gazebo
# window, which is why this doesn't affect anything visual) needs the NVIDIA EGL vendor
# forced explicitly on this NVIDIA-hybrid-graphics machine, or it silently renders
# blank/uniform camera frames -- no error most of the time, just wrong data. See
# PROGRESS.md Session 11 for how this was actually diagnosed (pixel content, not just
# "no crash").
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# --- 1. Simulation (junior_ctrl: real legs, real gait) ------------------------------
if ros2 topic list 2>/dev/null | grep -qx "/odom" && ros2 topic list 2>/dev/null | grep -qx "/scan"; then
    info "Simulation already running -- reusing it."
else
    info "Starting Gazebo simulation with the real walking controller..."
    ros2 launch unitree_go2_sim unitree_go2_nav_bringup.launch.py > "$LOG_DIR/sim.log" 2>&1 &
    CHILD_PIDS+=($!); disown
    if ! wait_for_ros_topic "/odom" 30 || ! wait_for_ros_topic "/scan" 30; then
        err "Simulation failed to come up. See $LOG_DIR/sim.log"
        exit 1
    fi
    info "Simulation is up (/odom, /scan publishing). Standing the robot up..."
    # auto_stand.py drives junior_ctrl's FSM Passive -> FixedStand -> MOVE_BASE; this
    # takes ~15-20s in practice (FixedStand alone is ~14s) -- Nav2 goals sent before
    # this finishes will just make the robot try to walk before it's balanced.
    if ! wait_for_log_pattern "$LOG_DIR/sim.log" "now listening to /cmd_vel" 45; then
        err "Robot never finished standing up. See $LOG_DIR/sim.log"
        exit 1
    fi
    info "Robot is standing and listening to /cmd_vel."
fi

# --- 2. Nav2 (localized against the saved junior map) -------------------------------
if ros2 node list 2>/dev/null | grep -qx "/amcl"; then
    info "Nav2 already running -- reusing it."
else
    info "Starting Nav2 (AMCL + planner + controller, using the saved junior map)..."
    ros2 launch go2_navigation nav2_junior.launch.py > "$LOG_DIR/nav2.log" 2>&1 &
    CHILD_PIDS+=($!); disown
    if ! wait_for_log_pattern "$LOG_DIR/nav2.log" "Managed nodes are active" 40; then
        err "Nav2 failed to come up. See $LOG_DIR/nav2.log"
        exit 1
    fi
    info "Nav2 is up."

    # Read the robot's REAL current position from /odom rather than assuming it's still
    # at the origin -- true right after a truly fresh sim launch, but wrong if an
    # existing simulation from an earlier run/session was reused.
    info "Reading current robot position for the initial pose..."
    read -r ODOM_X ODOM_Y ODOM_QZ ODOM_QW < <(python3 - <<'PYEOF'
import math
import rclpy
from nav_msgs.msg import Odometry
rclpy.init(args=None)
node = rclpy.create_node('initial_pose_helper')
result = {}
def cb(msg):
    result['x'] = msg.pose.pose.position.x
    result['y'] = msg.pose.pose.position.y
    # The real walking gait (junior_ctrl) has genuine roll/pitch sway in its base
    # orientation -- taking .z/.w directly from the full quaternion and dropping
    # .x/.y (as if it were a flat/planar robot) leaves a non-unit-magnitude
    # quaternion. AMCL's initial-pose validator (nav2_util::validateMsg) requires
    # magnitude == 1 and silently rejects anything else as "malformed", which
    # means AMCL never localizes, /map never gets a transform, and every
    # navigation goal is rejected -- confirmed live via nav2.log's repeated
    # "Received initialpose message is malformed. Rejecting." Fix: extract the
    # real yaw (atan2 on the full quaternion) and rebuild a clean, normalized
    # pure-yaw quaternion instead of truncating the original.
    o = msg.pose.pose.orientation
    yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
    result['qz'] = math.sin(yaw / 2.0)
    result['qw'] = math.cos(yaw / 2.0)
sub = node.create_subscription(Odometry, '/odom', cb, 10)
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.2)
    if result:
        break
node.destroy_node()
rclpy.shutdown()
print(result.get('x', 0.0), result.get('y', 0.0), result.get('qz', 0.0), result.get('qw', 1.0))
PYEOF
)
    ODOM_X="${ODOM_X:-0.0}"; ODOM_Y="${ODOM_Y:-0.0}"; ODOM_QZ="${ODOM_QZ:-0.0}"; ODOM_QW="${ODOM_QW:-1.0}"
    info "Setting initial pose to (x=$ODOM_X, y=$ODOM_Y) -- read live from /odom."
    timeout 3 ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $ODOM_X, y: $ODOM_Y, z: 0.0}, orientation: {z: $ODOM_QZ, w: $ODOM_QW}}, covariance: [0.25,0.0,0.0,0.0,0.0,0.0, 0.0,0.25,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.06]}}" \
        -r 5 > /dev/null 2>&1 || true
    sleep 2
fi

# --- 3. Object detection (camera + YOLO-World) ---------------------------------------
# GO2_DETECTION=off skips this (faster startup, no "what's in the room" capability).
if [ "${GO2_DETECTION:-on}" = "off" ]; then
    info "Object detection disabled (GO2_DETECTION=off)."
elif ros2 node list 2>/dev/null | grep -qx "/yolo/yolo_node"; then
    info "Object detection already running -- reusing it."
else
    info "Starting object detection (YOLO-World on the camera feed)..."
    ros2 launch yolo_bringup yolo-world.launch.py \
        input_image_topic:=/rgbd_camera/image \
        input_depth_topic:=/rgbd_camera/depth_image \
        input_depth_info_topic:=/rgbd_camera/camera_info \
        target_frame:=map threshold:=0.02 use_3d:=True use_tracking:=False \
        depth_image_units_divisor:=1 device:=cuda:0 \
        > "$LOG_DIR/yolo.log" 2>&1 &
    CHILD_PIDS+=($!); disown
    if ! wait_for_log_pattern "$LOG_DIR/yolo.log" "\[yolo_node\] Activated" 30; then
        warn "Object detection didn't come up in time -- continuing without it."
        warn "See $LOG_DIR/yolo.log. (\"what's in the room\" won't work this run.)"
    else
        # Default vocabulary: this world's known objects by color/shape (plain
        # primitives, not real furniture -- see PROGRESS.md Session 11 for why
        # YOLO-World instead of stock YOLO) plus a few generic fallbacks so it's not
        # useless in a world with different objects.
        timeout 60 ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses \
            "{classes: ['red box', 'blue box', 'green box', 'yellow cylinder', 'cyan cylinder', 'box', 'cylinder', 'chair', 'table', 'person']}" \
            > /dev/null 2>&1 || warn "Could not set detection classes -- see $LOG_DIR/yolo.log."
        info "Object detection is up."
    fi
fi

# --- 4. LLM (Gemini by default -- no local server to start) -------------------------
export GO2_LLM_API_KEY="${GO2_LLM_API_KEY:-$GEMINI_API_KEY}"
if [ -z "$GO2_LLM_API_KEY" ] && [ -z "${GO2_LLM_BASE_URL:-}" ]; then
    err "No Gemini API key found. Get a free one at https://aistudio.google.com/app/apikey"
    err "then: export GEMINI_API_KEY=your-key-here"
    exit 1
fi
export GO2_LLM_MODEL="${GO2_LLM_MODEL:-gemini-3.1-flash-lite}"
info "Using LLM model: $GO2_LLM_MODEL"

# --- 5. Hand off to the chat interface ----------------------------------------------
export GO2_PLACES_FILE="$COURSE_DIR/src/go2_semantic_map/config/places_junior.yaml"
echo
echo -e "${BOLD}Everything is up. Known places you can ask the robot to go to:${NC}"
python3 - <<PYEOF 2>/dev/null || echo "  (waypoint, start)"
from go2_semantic_map.places import load_places
for name, p in load_places("$GO2_PLACES_FILE").items():
    print(f"  - {name}: {p.description}")
PYEOF
echo

# GO2_UI=web for the browser tab instead, GO2_UI=cli for the plain terminal chat.
UI="${GO2_UI:-gui}"
# Deliberately not `exec`'d: this needs to return to the script afterwards so the
# EXIT trap runs and cleans up the simulation/Nav2 processes started above when
# the user quits the chat.
if [ "$UI" = "cli" ]; then
    echo -e "${BOLD}Try things like:${NC} \"go to the waypoint\", \"what's the status?\", \"stop\", \"quit\" to exit."
    echo
    ros2 run go2_llm_nav go2_llm_nav
elif [ "$UI" = "web" ]; then
    info "Opening the chat GUI in your browser..."
    ros2 run go2_llm_nav go2_llm_nav_web
else
    info "Opening the chat GUI window..."
    ros2 run go2_llm_nav go2_llm_nav_gui
fi
