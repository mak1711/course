#!/usr/bin/env python3
"""Drive junior_ctrl's FSM from Passive -> FixedStand -> MOVE_BASE automatically.

junior_ctrl (unitree_guide2) only accepts state-transition commands via a raw-TTY
keyboard reader (interface/KeyBoard.cpp) -- there is no ROS topic/service for it. That's
fine for a human testing it interactively, but Nav2 needs the robot to already be
standing and in the MOVE_BASE state (the one FSM state that subscribes to /cmd_vel --
see FSM/State_move_base.cpp) before it can drive it at all.

This spawns `ros2 run unitree_guide2 junior_ctrl` attached to a pty (so its termios raw
keyboard reader has a real terminal to read from), waits for it to settle in Passive,
sends '2' (FixedStand) and waits for the FSM's own telemetry to report completion
(`percent=1.000`), then sends '5' (MOVE_BASE) so it starts listening to /cmd_vel.
Everything junior_ctrl prints is relayed to this process's stdout so it still shows up
in `ros2 launch` output / logs.
"""

import os
import pty
import re
import select
import subprocess
import sys
import time

FIXEDSTAND_KEY = b"2"
MOVE_BASE_KEY = b"5"
SETTLE_WAIT_S = 3.0
STAND_TIMEOUT_S = 30.0  # observed FixedStand takes ~14s; generous margin
STAND_DONE_RE = re.compile(r"percent=1\.000")


def relay_until(master_fd, deadline, pattern=None):
    """Relay junior_ctrl's output to our stdout until `deadline` (epoch seconds) or,
    if `pattern` is given, until it matches -- whichever comes first. Returns True if
    the pattern matched, False on timeout/EOF."""
    buf = b""
    while time.time() < deadline:
        r, _, _ = select.select([master_fd], [], [], 0.5)
        if master_fd not in r:
            continue
        try:
            chunk = os.read(master_fd, 4096)
        except OSError:
            return False
        if not chunk:
            return False
        sys.stdout.buffer.write(chunk)
        sys.stdout.flush()
        buf += chunk
        if len(buf) > 8192:
            buf = buf[-8192:]
        if pattern and pattern.search(buf.decode(errors="ignore")):
            return True
    return False


def main() -> int:
    master_fd, slave_fd = pty.openpty()
    proc = subprocess.Popen(
        ["ros2", "run", "unitree_guide2", "junior_ctrl"],
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        close_fds=True,
    )
    os.close(slave_fd)

    try:
        relay_until(master_fd, time.time() + SETTLE_WAIT_S)

        print("\n[auto_stand] sending FixedStand (key '2')", file=sys.stderr)
        os.write(master_fd, FIXEDSTAND_KEY)
        ok = relay_until(master_fd, time.time() + STAND_TIMEOUT_S, STAND_DONE_RE)
        if not ok:
            print(
                "[auto_stand] WARNING: never saw percent=1.000 within "
                f"{STAND_TIMEOUT_S}s -- sending MOVE_BASE anyway, but the robot may "
                "not actually be standing.",
                file=sys.stderr,
            )
        time.sleep(1.0)

        print(
            "[auto_stand] sending MOVE_BASE (key '5') -- now listening to /cmd_vel",
            file=sys.stderr,
        )
        os.write(master_fd, MOVE_BASE_KEY)

        relay_until(master_fd, float("inf"))
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    return proc.returncode or 0


if __name__ == "__main__":
    sys.exit(main())
