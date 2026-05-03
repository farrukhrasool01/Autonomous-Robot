"""Autonomous navigation step for the Husarion RosBot.

Call autonomous_step(block_timer) once per simulation step.
Returns (v_cmd, omega_cmd, label, new_block_timer, debug).

All sensor reads happen here; no Webots imports elsewhere in this file.
"""

import math

import devices
import sensors
from reactive import laser_5_sectors, wall_follow_twist
from config import (
    TARGET_LIN_VEL, TARGET_ANG_VEL,
    FRONT_STOP_DIST, FRONT_BLOCK_DIST, FRONT_CAUTION_DIST,
    CAUTION_LIN_VEL, CAUTION_ANG_VEL,
    WALL_TARGET_DIST, WALL_CLOSE_BAND, WALL_LOST_DIST, SIDE_DANGER_DIST,
    OMEGA_SMALL, BLOCK_TIMEOUT, REAR_SAFE_DIST,
    DEPTH_CAUTION_DIST, DEPTH_FRONT_BLOCK_DIST, DEPTH_FRONT_STOP_DIST,
)


def autonomous_step(block_timer):
    """Run one step of autonomous right-hand wall-following.

    Parameters
    ----------
    block_timer : int
        Consecutive steps the front has been blocked (carried across calls).

    Returns
    -------
    (v_cmd, omega_cmd, label, new_block_timer, debug) : tuple
    """
    # ── Laser sectors ──────────────────────────────────────────────────────────
    raw_ranges = []
    if devices.laser:
        try:
            raw_ranges = devices.laser.getRangeImage() or []
        except Exception:
            pass
    _, left_min, center_min, right_min, _ = laser_5_sectors(raw_ranges)

    # ── Short-range sensors ────────────────────────────────────────────────────
    fl_val = devices.fl_range.getValue() if devices.fl_range else float('inf')
    fr_val = devices.fr_range.getValue() if devices.fr_range else float('inf')
    rl_val = devices.rl_range.getValue() if devices.rl_range else float('inf')
    rr_val = devices.rr_range.getValue() if devices.rr_range else float('inf')

    rear_safe = rl_val >= REAR_SAFE_DIST and rr_val >= REAR_SAFE_DIST

    # ── Depth camera ───────────────────────────────────────────────────────────
    depth_front   = sensors.read_depth_front_min()
    depth_valid   = math.isfinite(depth_front)
    depth_caution = depth_valid and depth_front < DEPTH_CAUTION_DIST
    depth_blocked = depth_valid and depth_front < DEPTH_FRONT_BLOCK_DIST
    depth_emg     = depth_valid and depth_front < DEPTH_FRONT_STOP_DIST

    # ── Block-timer (depth caution alone does NOT count as a block) ────────────
    laser_blocked = center_min < FRONT_BLOCK_DIST
    if laser_blocked or depth_blocked:
        block_timer += 1
    else:
        block_timer = 0

    # ── Core twist selection ───────────────────────────────────────────────────
    if laser_blocked or depth_blocked:
        # Rotate toward whichever side has more clearance
        open_sign = 1.0 if left_min >= right_min else -1.0
        v_cmd     = 0.0
        omega_cmd = open_sign * TARGET_ANG_VEL
        label     = "block_turn_left" if open_sign > 0 else "block_turn_right"

    else:
        # Right-hand wall following; fused front = laser centre only
        v_cmd, omega_cmd, label = wall_follow_twist(
            center_min, right_min, left_min,
            block_timer, rear_safe,
            FRONT_BLOCK_DIST, FRONT_CAUTION_DIST,
            WALL_TARGET_DIST, WALL_CLOSE_BAND, WALL_LOST_DIST, SIDE_DANGER_DIST,
            BLOCK_TIMEOUT,
            TARGET_LIN_VEL, CAUTION_LIN_VEL,
            # wall_follow_twist signature: omega_small, omega_caution, omega_large
            OMEGA_SMALL, CAUTION_ANG_VEL, TARGET_ANG_VEL,
        )

        # Depth caution: cap forward speed without triggering recovery
        if depth_caution and v_cmd > 0:
            v_cmd  = min(v_cmd, 0.07)
            label += "_depth_caution"

    # ── Hard emergency overrides (applied after wall-follow decision) ──────────
    laser_front_emg = center_min < FRONT_STOP_DIST
    fl_emg    = fl_val < FRONT_STOP_DIST
    fr_emg    = fr_val < FRONT_STOP_DIST
    front_emg = laser_front_emg or fl_emg or fr_emg

    if v_cmd > 0 and front_emg:
        v_cmd = 0.0
        if fl_emg and not fr_emg:
            omega_cmd = -TARGET_ANG_VEL
            label     = "front_left_emg_turn_right"
        elif fr_emg and not fl_emg:
            omega_cmd = TARGET_ANG_VEL
            label     = "front_right_emg_turn_left"
        else:
            omega_cmd = TARGET_ANG_VEL
            label     = "front_emg_turn_left"

    if v_cmd > 0 and depth_emg:
        v_cmd     = 0.0
        omega_cmd = TARGET_ANG_VEL
        label     = "depth_emg_turn_left"

    # ── Rear safety: block reverse when rear sensors are close ─────────────────
    if v_cmd < 0 and (rl_val < REAR_SAFE_DIST or rr_val < REAR_SAFE_DIST):
        v_cmd     = 0.0
        omega_cmd = TARGET_ANG_VEL
        label     = "rear_blocked_turn_left"

    debug = {
        "depth_front":   depth_front,
        "depth_caution": depth_caution,
        "depth_blocked": depth_blocked,
        "depth_emg":     depth_emg,
        "block_timer":   block_timer,
    }
    return v_cmd, omega_cmd, label, block_timer, debug
