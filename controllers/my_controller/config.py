"""Tunable control constants for the Husarion RosBot controller.

All physical values are in SI units (metres, radians, seconds).
Adjust these to tune robot behaviour without touching logic files.
"""

# ── Motion speeds ────────────────────────────────────────────────────────────
TARGET_LIN_VEL = 0.05   # m/s   — nominal forward speed
TARGET_ANG_VEL = 0.50   # rad/s — nominal rotation rate

# ── Laser front safety thresholds ────────────────────────────────────────────
FRONT_STOP_DIST    = 0.15  # m — hard emergency stop (laser + range sensors)
FRONT_BLOCK_DIST   = 0.25  # m — robot is blocked; stop forward and rotate
FRONT_CAUTION_DIST = 0.35  # m — caution zone entry; slow down and steer

# ── Caution zone speeds ───────────────────────────────────────────────────────
CAUTION_LIN_VEL = 0.04  # m/s   — reduced forward speed in caution zone
CAUTION_ANG_VEL = 0.08  # rad/s — steering rate in caution zone

# ── Wall following ────────────────────────────────────────────────────────────
WALL_TARGET_DIST = 0.35  # m     — desired right-wall following distance
WALL_CLOSE_BAND  = 0.12  # m     — dead-band half-width around target
WALL_LOST_DIST   = 0.80  # m     — right distance above which wall is considered lost
SIDE_DANGER_DIST = 0.05  # m     — side clearance below which danger steering activates
OMEGA_SMALL      = 0.22  # rad/s — gentle wall-correction angular rate

# ── Recovery ─────────────────────────────────────────────────────────────────
BLOCK_TIMEOUT  = 40    # steps — front-blocked steps before timeout recovery activates
REAR_SAFE_DIST = 0.25  # m     — rear range below which reverse is forbidden

# ── Depth camera safety ───────────────────────────────────────────────────────
DEPTH_CAUTION_DIST     = 0.60  # m — depth caution zone (slow down)
DEPTH_FRONT_BLOCK_DIST = 0.45  # m — depth hard block (stop and rotate)
DEPTH_FRONT_STOP_DIST  = 0.10  # m — depth emergency stop
DEPTH_MIN_VALID        = 0.10  # m — minimum valid depth pixel value
