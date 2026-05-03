"""Motion helpers for the Husarion RosBot.

Provides drive_twist() as the single entry point for commanding movement.
All wheel-speed writes go through set_wheel_speeds() which enforces the
per-wheel saturation limit from kinematics.py.
"""

import devices
from kinematics import MAX_WHEEL_SPEED_RAD_S, clamp_twist, wheel_speeds_from_twist


def set_wheel_speeds(left, right):
    """Write angular velocities (rad/s) to all four wheel motors."""
    left  = max(-MAX_WHEEL_SPEED_RAD_S, min(MAX_WHEEL_SPEED_RAD_S, left))
    right = max(-MAX_WHEEL_SPEED_RAD_S, min(MAX_WHEEL_SPEED_RAD_S, right))
    devices.fl_motor.setVelocity(left)
    devices.rl_motor.setVelocity(left)
    devices.fr_motor.setVelocity(right)
    devices.rr_motor.setVelocity(right)


def drive_twist(v, omega):
    """Command a body twist (m/s, rad/s).

    Clamps the twist so no wheel saturates, converts to wheel angular
    velocities, and writes to the motors.

    Returns
    -------
    (v_real, omega_real, wL, wR) : realized values after clamping
    """
    v_c, w_c = clamp_twist(v, omega)
    left, right = wheel_speeds_from_twist(v_c, w_c)
    set_wheel_speeds(left, right)
    return v_c, w_c, left, right


def stop_robot():
    set_wheel_speeds(0.0, 0.0)
