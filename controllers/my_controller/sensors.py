"""Sensor-reading helpers for the Husarion RosBot.

All functions read from the hardware handles in devices.py.
Pure data functions — no control logic.
"""

import math

import devices
from config import DEPTH_MIN_VALID


def get_front_laser_min():
    """Minimum range in the laser's front-centre band (40% to 60% of rays)."""
    if devices.laser is None:
        return float('inf')
    try:
        ranges = devices.laser.getRangeImage()
    except Exception:
        return float('inf')
    if not ranges:
        return float('inf')
    n     = len(ranges)
    start = int(0.4 * n)
    end   = int(0.6 * n)
    front = [r for r in ranges[start:end] if r > 0.0 and math.isfinite(r)]
    return min(front) if front else float('inf')


def read_depth_front_min():
    """5th-percentile depth in the front-centre ROI (floating-obstacle detection).

    ROI: centre third of width, upper two-thirds of height, every 2nd pixel.
    Returns float('inf') when depth is unavailable or the ROI has no valid pixels.
    """
    if devices.camera_depth is None:
        return float('inf')
    try:
        w    = devices.camera_depth.getWidth()
        h    = devices.camera_depth.getHeight()
        data = devices.camera_depth.getRangeImage()
    except Exception:
        return float('inf')
    if not data or w == 0 or h == 0:
        return float('inf')

    col_start = w // 3
    col_end   = 2 * w // 3
    row_end   = 2 * h // 3

    vals = []
    for row in range(0, row_end, 2):
        base = row * w
        for col in range(col_start, col_end, 2):
            v = data[base + col]
            if math.isfinite(v) and v > DEPTH_MIN_VALID:
                vals.append(v)

    if not vals:
        return float('inf')

    vals.sort()
    idx = max(0, len(vals) // 20)  # 5th percentile
    return vals[idx]


def read_sensor_snapshot():
    """Read every enabled sensor and return a readings dict for sensor_debug."""
    r = {}

    # Wheel encoders
    for key, sensor in [
        ("enc_fl", devices.fl_wheel_sensor), ("enc_fr", devices.fr_wheel_sensor),
        ("enc_rl", devices.rl_wheel_sensor), ("enc_rr", devices.rr_wheel_sensor),
    ]:
        try:
            r[key] = sensor.getValue() if sensor else None
        except Exception:
            r[key] = None

    # IMU — getValues() for accel/gyro/compass, getRollPitchYaw() for inertial_unit
    for key, sensor, method in [
        ("accel",   devices.accelerometer, "getValues"),
        ("gyro",    devices.gyro,          "getValues"),
        ("compass", devices.compass,       "getValues"),
        ("imu_rpy", devices.inertial_unit, "getRollPitchYaw"),
    ]:
        try:
            r[key] = tuple(getattr(sensor, method)()) if sensor else None
        except Exception:
            r[key] = None

    # Range sensors
    for key, sensor in [
        ("range_fl", devices.fl_range), ("range_fr", devices.fr_range),
        ("range_rl", devices.rl_range), ("range_rr", devices.rr_range),
    ]:
        try:
            r[key] = sensor.getValue() if sensor else None
        except Exception:
            r[key] = None

    # Laser
    if devices.laser:
        try:
            data = devices.laser.getRangeImage()
            if data:
                finite = [v for v in data if math.isfinite(v) and v > 0.0]
                r["laser_count"] = len(data)
                r["laser_min"]   = min(finite) if finite else float("inf")
                r["laser_max"]   = max(finite) if finite else 0.0
                r["laser_mean"]  = sum(finite) / len(finite) if finite else float("nan")
            else:
                r["laser_count"] = 0
        except Exception:
            r["laser_count"] = None
    else:
        r["laser_count"] = None

    # RGB camera — image is BGRA byte order in Webots
    if devices.camera_rgb:
        try:
            w   = devices.camera_rgb.getWidth()
            h   = devices.camera_rgb.getHeight()
            img = devices.camera_rgb.getImage()
            idx = 4 * (h // 2 * w + w // 2)
            b, g, rv = img[idx], img[idx + 1], img[idx + 2]
            r["cam_rgb"] = (w, h, (rv, g, b))
        except Exception:
            r["cam_rgb"] = None
    else:
        r["cam_rgb"] = None

    # Depth camera
    if devices.camera_depth:
        try:
            w          = devices.camera_depth.getWidth()
            h          = devices.camera_depth.getHeight()
            depth_data = devices.camera_depth.getRangeImage()
            ctr        = depth_data[h // 2 * w + w // 2] if depth_data else None
            r["cam_depth"] = (w, h, ctr)
        except Exception:
            r["cam_depth"] = None
    else:
        r["cam_depth"] = None

    return r
