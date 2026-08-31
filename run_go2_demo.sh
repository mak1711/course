#!/bin/bash
# One-command launcher for the Go2 natural-language navigation demo.
#
# Brings up: the Gazebo simulation, Nav2 (localized against the saved map of the demo
# room), and finally drops you into an interactive chat prompt where you can type
# things like "go to the sofa" and the robot will actually walk there. Talks to Gemini
# for the LLM (needs a free API key -- see USAGE.md); no local model server to start.
#
# Usage: ./run_go2_demo.sh
# Stop:  Ctrl-C (cleans up everything it started)

# Note: no `set -u` -- ROS 2's own setup.bash references unbound variables internally,
# so nounset mode breaks sourcing it. -o pipefail alone is safe here.
set -o pipefail

COURSE_DIR="/home/kan/lab/course"
LOG_DIR="/tmp/go2_demo_logs"
mkdir -p "$LOG_DIR"

GREEN='\033[0;32m'; YELLOW='\033[0;33m'; RED='\033[0;31m'; BOLD='\033[1m'; NC='\033[0m'
info()  { echo -e "${GREEN}[go2-demo]${NC} $1"; }
warn()  { echo -e "${YELLOW}[go2-demo]${NC} $1"; }
err()   { echo -e "${RED}[go2-demo]${NC} $1"; }

CHILD_PIDS=()
cleanup() {
    echo
    info "Shutting down everything this script started..."
    for pid in "${CHILD_PIDS[@]:-}"; do
        kill -9 "$pid" >/dev/null 2>&1 || true
    done
    # ros2 launch spawns its own process tree; make sure nothing lingers. Note: the
    # actual long-running "ign gazebo server"/"ign gazebo gui" child processes don't
    # carry the world path or any launch-script name in their argv (only the short-
    # lived wrapper that spawns them does) -- match on "ign gazebo" alone or these
    # survive cleanup and are left running after the script exits.
    pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
    pkill -9 -f "nav2_container" >/dev/null 2>&1 || true
    pkill -9 -f "rviz2" >/dev/null 2>&1 || true
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

echo -e "${BOLD}=== Go2 natural-language navigation demo ===${NC}"
echo

source /opt/ros/humble/setup.bash
source "$COURSE_DIR/install/setup.bash"

# --- 1. Simulation -----------------------------------------------------------------
if ros2 topic list 2>/dev/null | grep -qx "/odom" && ros2 topic list 2>/dev/null | grep -qx "/scan"; then
    info "Simulation already running -- reusing it."
else
    info "Starting Gazebo simulation..."
    ros2 launch go2_simulation sim.launch.py > "$LOG_DIR/sim.log" 2>&1 &
    CHILD_PIDS+=($!); disown
    if ! wait_for_ros_topic "/odom" 30 || ! wait_for_ros_topic "/scan" 30; then
        err "Simulation failed to come up. See $LOG_DIR/sim.log"
        exit 1
    fi
    info "Simulation is up (/odom, /scan publishing)."
fi

# --- 2. Nav2 (localized against the saved map) -----------------------------------
if ros2 node list 2>/dev/null | grep -qx "/amcl"; then
    info "Nav2 already running -- reusing it."
else
    info "Starting Nav2 (AMCL + planner + controller, using the saved map)..."
    ros2 launch go2_navigation nav2.launch.py > "$LOG_DIR/nav2.log" 2>&1 &
    CHILD_PIDS+=($!); disown
    if ! wait_for_log_pattern "$LOG_DIR/nav2.log" "Managed nodes are active" 40; then
        err "Nav2 failed to come up. See $LOG_DIR/nav2.log"
        exit 1
    fi
    info "Nav2 is up."

    # Read the robot's REAL current position from /odom rather than assuming it's still
    # at the origin -- true right after a truly fresh sim launch, but wrong if an
    # existing simulation from an earlier run/session was reused (see "already running --
    # reusing it" above), which caused AMCL to be told the wrong starting position and
    # every subsequent goal to get rejected. go2_simulation's odometry is kinematic
    # (no wheel slip), so it's accurate enough to trust directly here.
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
    # Extract yaw from the full quaternion and rebuild a clean, normalized
    # pure-yaw quaternion rather than taking .z/.w directly -- if roll/pitch
    # ever aren't exactly zero, truncating them leaves a non-unit-magnitude
    # quaternion, which AMCL's initial-pose validator silently rejects as
    # "malformed" (confirmed the hard way on the junior_ctrl walking robot's
    # real gait sway -- see run_go2_demo_junior.sh). Harmless here even though
    # this robot's odometry is planar, and keeps both scripts consistent.
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

# --- 3. LLM (Gemini by default -- no local server to start) -------------------------
export GO2_LLM_API_KEY="${GO2_LLM_API_KEY:-$GEMINI_API_KEY}"
if [ -z "$GO2_LLM_API_KEY" ] && [ -z "${GO2_LLM_BASE_URL:-}" ]; then
    err "No Gemini API key found. Get a free one at https://aistudio.google.com/app/apikey"
    err "then: export GEMINI_API_KEY=your-key-here"
    exit 1
fi
export GO2_LLM_MODEL="${GO2_LLM_MODEL:-gemini-3.1-flash-lite}"
info "Using LLM model: $GO2_LLM_MODEL"

# --- 4. Hand off to the chat interface ----------------------------------------------
echo
echo -e "${BOLD}Everything is up. Known places you can ask the robot to go to:${NC}"
python3 - <<'PYEOF' 2>/dev/null || echo "  (sofa, table, entrance)"
from ament_index_python.packages import get_package_share_directory
from go2_semantic_map.places import load_places
for name, p in load_places().items():
    print(f"  - {name}: {p.description}")
PYEOF
echo

# GO2_UI=web for the browser tab instead, GO2_UI=cli for the plain terminal chat.
UI="${GO2_UI:-gui}"
# Deliberately not `exec`'d: this needs to return to the script afterwards so the
# EXIT trap runs and cleans up the simulation/Nav2 processes started above when the
# user quits the chat.
if [ "$UI" = "cli" ]; then
    echo -e "${BOLD}Try things like:${NC} \"go to the sofa\", \"what's the status?\", \"stop\", \"quit\" to exit."
    echo
    ros2 run go2_llm_nav go2_llm_nav
elif [ "$UI" = "web" ]; then
    info "Opening the chat GUI in your browser..."
    ros2 run go2_llm_nav go2_llm_nav_web
else
    info "Opening the chat GUI window..."
    ros2 run go2_llm_nav go2_llm_nav_gui
fi
