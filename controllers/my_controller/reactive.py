"""Local safe-path planner for the Husarion RosBot.

Pure Python — no Webots imports.  Receives already-read sensor data and
returns a body twist (v, omega).

Laser indexing convention (verified working in M3 / M3.0b):
  Ray index 0 = leftmost ray; index n-1 = rightmost ray (robot's view).
  Five equal sectors of n//5 rays:

    far_left   [0,      n//5)     broad left arc
    left       [n//5,   2*n//5)   front-left quadrant
    center     [2*n//5, 3*n//5)   straight ahead      ← matches get_front_laser_min()
    right      [3*n//5, 4*n//5)   front-right quadrant
    far_right  [4*n//5, n)        broad right arc

  The center sector [2n/5, 3n/5] equals [0.4n, 0.6n], which is the same
  window used by get_front_laser_min() — the empirically validated front
  direction.  Only the outer two fifths on each side are new.
"""

import math

# ---------------------------------------------------------------------------
# Candidate commands
# (label, v_scale, omega_scale) — actual values = scale × lin_vel / ang_vel
# ---------------------------------------------------------------------------

CANDIDATES = [
    ('forward',       1.0,   0.0  ),
    ('slight_left',   1.0,  +0.35 ),
    ('slight_right',  1.0,  -0.35 ),
    ('strong_left',   0.5,  +1.0  ),
    ('strong_right',  0.5,  -1.0  ),
    ('rotate_left',   0.0,  +1.0  ),
    ('rotate_right',  0.0,  -1.0  ),
]

# ---------------------------------------------------------------------------
# Scoring weights  (tune these to adjust planner behaviour)
# ---------------------------------------------------------------------------

W_CLEARANCE   = 3.0   # reward: open space in the candidate's direction
W_FORWARD     = 5.0   # reward: positive forward velocity
W_TURN        = 1.5   # penalty: magnitude of angular velocity
W_REPEAT      = 2.5   # penalty: repeating the same omega-sign direction
CLEARANCE_CAP = 2.0   # metres — clearance reward saturates above this

# ---------------------------------------------------------------------------
# Five-sector laser extraction
# ---------------------------------------------------------------------------

def laser_5_sectors(ranges):
    """Split a laser range image into five directional sectors.

    Returns (far_left_min, left_min, center_min, right_min, far_right_min).
    Any sector with no finite positive ray returns inf.
    Gracefully handles None or empty ranges.
    """
    if not ranges:
        return (float('inf'),) * 5

    n = len(ranges)

    def _min(start, end):
        vals = [r for r in ranges[start:end] if math.isfinite(r) and r > 0.0]
        return min(vals) if vals else float('inf')

    s = max(1, n // 5)
    return (
        _min(0,     s  ),   # far_left
        _min(s,   2*s  ),   # left
        _min(2*s, 3*s  ),   # center  (= [0.4n, 0.6n], same as get_front_laser_min)
        _min(3*s, 4*s  ),   # right
        _min(4*s,   n  ),   # far_right (may be slightly wider when n % 5 != 0)
    )


# ---------------------------------------------------------------------------
# Candidate clearance mapping
# ---------------------------------------------------------------------------

def candidate_clearance(v_scale, omega_scale, sectors):
    """Return the minimum clearance (m) in the direction a candidate travels.

    Forward candidates check the sector(s) in their arc path.
    Rotation candidates check the far sector on their turn side.
    Reverse is assumed clear (no rear laser on the RosBot).

    sectors = (far_left, left, center, right, far_right)
    """
    far_left, left_z, center, right_z, far_right = sectors

    if v_scale > 0:
        if omega_scale > 0.5:      # strong arc left  (omega_scale == 1.0)
            return left_z
        elif omega_scale > 0:      # slight arc left  (omega_scale == 0.35)
            return min(center, left_z)
        elif omega_scale < -0.5:   # strong arc right
            return right_z
        elif omega_scale < 0:      # slight arc right
            return min(center, right_z)
        else:                      # straight forward
            return center
    elif v_scale < 0:              # reverse — no rear sensor
        return float('inf')
    else:
        if omega_scale > 0:        # rotate left in place
            return far_left
        else:                      # rotate right in place
            return far_right


# ---------------------------------------------------------------------------
# Candidate scoring
# ---------------------------------------------------------------------------

def score_candidate(v_scale, omega_scale, sectors, recent_omegas,
                    fl_emergency, fr_emergency, stop_dist, lin_vel, ang_vel):
    """Score one candidate twist.  Returns a float, or -inf if unsafe.

    Parameters
    ----------
    v_scale, omega_scale : float
        Scale factors from CANDIDATES.
    sectors : tuple[float, float, float, float, float]
        Output of laser_5_sectors().
    recent_omegas : list[int]
        Recent omega signs (+1, 0, -1), most recent last.
        The controller maintains this as a deque and passes list(deque).
    fl_emergency, fr_emergency : bool
        True when the corresponding range sensor reads < stop_dist.
    stop_dist, lin_vel, ang_vel : float
        Control constants from the controller.
    """
    v_actual     = v_scale     * lin_vel
    omega_actual = omega_scale * ang_vel
    clearance    = candidate_clearance(v_scale, omega_scale, sectors)

    # Hard safety gates — these candidates are not evaluated further
    if v_actual > 0 and clearance < stop_dist:
        return float('-inf')
    if v_actual > 0 and (fl_emergency or fr_emergency):
        return float('-inf')

    # Clearance score, capped so very open space doesn't dominate
    clr_score = min(clearance, CLEARANCE_CAP)

    # Repeat penalty: how much of recent history shares this omega sign?
    repeat_pen = 0.0
    if recent_omegas:
        omega_sign = 1 if omega_actual > 0 else (-1 if omega_actual < 0 else 0)
        if omega_sign != 0:
            same = sum(1 for s in recent_omegas if s == omega_sign)
            repeat_pen = same / len(recent_omegas)

    return (
          W_CLEARANCE * clr_score
        + W_FORWARD   * v_actual
        - W_TURN      * abs(omega_actual)
        - W_REPEAT    * repeat_pen
    )


# ---------------------------------------------------------------------------
# Local planner — main entry point
# ---------------------------------------------------------------------------

def local_plan(sectors, recent_omegas, fl_emergency, fr_emergency,
               stop_dist, lin_vel, ang_vel):
    """Evaluate all candidates and return the best safe twist.

    Returns
    -------
    (v, omega, label) : (float, float, str)
        Best commanded twist and its label for logging.
        v and omega are in m/s and rad/s respectively.

    If every candidate is unsafe (surrounded), falls back to rotating toward
    whichever far sector has more clearance.  This fallback cannot return
    -inf because rotation candidates are never blocked by the safety gate
    (they have v_actual == 0).
    """
    best_score = float('-inf')
    best_label = None
    best_v     = 0.0
    best_omega = 0.0

    for label, v_scale, omega_scale in CANDIDATES:
        s = score_candidate(
            v_scale, omega_scale, sectors, recent_omegas,
            fl_emergency, fr_emergency, stop_dist, lin_vel, ang_vel,
        )
        if s > best_score:
            best_score = s
            best_label = label
            best_v     = v_scale * lin_vel
            best_omega = omega_scale * ang_vel

    if best_score == float('-inf'):
        # Completely surrounded — rotate toward the more open far sector
        far_left, _, _, _, far_right = sectors
        if far_left >= far_right:
            return 0.0, ang_vel, 'fallback_left'
        else:
            return 0.0, -ang_vel, 'fallback_right'

    return best_v, best_omega, best_label



def wall_follow_twist(front_min, right_min, left_min,
                      block_timer, rear_safe,
                      front_block, front_caution,
                      wall_target, wall_band, wall_lost, side_danger,
                      block_timeout,
                      v_fwd, v_caution,
                      omega_small, omega_caution, omega_large):
    """Right-hand wall-following with front caution and open-side turning."""

    # Decide which side is more open
    # Positive omega = turn left
    # Negative omega = turn right
    if left_min > right_min:
        open_side_omega = omega_large
        open_side_omega_caution = omega_caution
        open_side_label = "left"
    else:
        open_side_omega = -omega_large
        open_side_omega_caution = -omega_caution
        open_side_label = "right"

    # 1. Front truly blocked
    if front_min <= front_block:
        if block_timer >= block_timeout:
            if rear_safe:
                return -v_fwd, 0.0, "recovery_backup"
            else:
                return 0.0, open_side_omega, f"recovery_rotate_{open_side_label}"

        return 0.0, open_side_omega, f"front_block_turn_{open_side_label}"

    # 2. Front caution zone: narrow but maybe passable
    if front_min <= front_caution:
        return v_caution, open_side_omega_caution, f"front_caution_turn_{open_side_label}"

    # 3. Left side danger
    if left_min < side_danger:
        return v_fwd * 0.5, -omega_small, "avoid_left"

    # 4. Right side danger
    if right_min < side_danger:
        return v_fwd * 0.5, omega_small, "avoid_right"

    # 5. Right wall lost
    if right_min > wall_lost:
        return v_fwd, -omega_small, "seek_wall"

    # 6. Right wall too far
    if right_min > wall_target + wall_band:
        return v_fwd, -omega_small, "correct_right"

    # 7. Right wall too close
    if right_min < wall_target - wall_band:
        return v_fwd, omega_small, "correct_left"

    # 8. Good corridor
    return v_fwd, 0.0, "forward"
