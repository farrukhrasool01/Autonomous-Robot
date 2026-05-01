"""Keyboard-controlled robot controller for Webots.

Use the ASDF keys to drive the robot:
 - F: forward
 - S: backward
 - A: rotate left (in-place)
 - D: rotate right (in-place)

This replaces the previous autonomous loop with a simple teleop that
maps key presses to differential wheel speeds. It also includes a
small safety check using the front laser to prevent driving into close
obstacles.
"""

from controller import Robot, Keyboard
import math

from kinematics import (
    MAX_WHEEL_SPEED_RAD_S,
    clamp_twist,
    wheel_speeds_from_twist,
)
from sensor_debug import format_compact_sensors, format_sensor_snapshot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------------------- helpers ----------------------
def clamp(value, low, high):
    return max(low, min(high, value))

# ---------------------- motors -----------------------
def get_device_checked(name):
    dev = robot.getDevice(name)
    if dev is None:
        print(f"WARNING: device named '{name}' not found on robot model")
    return dev

# motor device names (adjust these if your proto uses different names)
fl_motor = get_device_checked("fl_wheel_joint")
fr_motor = get_device_checked("fr_wheel_joint")
rl_motor = get_device_checked("rl_wheel_joint")
rr_motor = get_device_checked("rr_wheel_joint")

for motor in [fl_motor, fr_motor, rl_motor, rr_motor]:
    if motor is None:
        continue
    try:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
    except Exception as e:
        print(f"Error configuring motor {motor}: {e}")

# ---------------------- sensors ----------------------
# wheel encoders
fl_wheel_sensor = get_device_checked("front left wheel motor sensor")
fr_wheel_sensor = get_device_checked("front right wheel motor sensor")
rl_wheel_sensor = get_device_checked("rear left wheel motor sensor")
rr_wheel_sensor = get_device_checked("rear right wheel motor sensor")

for sensor in [fl_wheel_sensor, fr_wheel_sensor, rl_wheel_sensor, rr_wheel_sensor]:
    if sensor is None:
        continue
    try:
        sensor.enable(timestep)
    except Exception:
        print(f"Could not enable sensor {sensor}")

# IMU
accelerometer = get_device_checked("imu accelerometer")
gyro = get_device_checked("imu gyro")
compass = get_device_checked("imu compass")
inertial_unit = get_device_checked("imu inertial_unit")

for s in (accelerometer, gyro, compass, inertial_unit):
    if s is None:
        continue
    try:
        s.enable(timestep)
    except Exception:
        print(f"Could not enable IMU device {s}")

# short-range range sensors
fl_range = get_device_checked("fl_range")
fr_range = get_device_checked("fr_range")
rl_range = get_device_checked("rl_range")
rr_range = get_device_checked("rr_range")

for sensor in [fl_range, fr_range, rl_range, rr_range]:
    if sensor is None:
        continue
    try:
        sensor.enable(timestep)
    except Exception:
        print(f"Could not enable short-range sensor {sensor}")

# cameras (if present)
camera_rgb = get_device_checked("camera rgb")
camera_depth = get_device_checked("camera depth")
if camera_rgb:
    try:
        camera_rgb.enable(timestep)
    except Exception:
        print("Could not enable camera_rgb")
if camera_depth:
    try:
        camera_depth.enable(timestep)
    except Exception:
        print("Could not enable camera_depth")

# laser (used for simple forward safety)
laser = get_device_checked("laser")
if laser:
    try:
        laser.enable(timestep)
    except Exception:
        print("Could not enable laser")

# ---------------------- keyboard ---------------------
keyboard = robot.getKeyboard()
if keyboard is None:
    print("WARNING: keyboard device not available")
else:
    keyboard.enable(timestep)
    print("Keyboard device enabled for controller")

# ---------------------- motion helpers ----------------
def set_wheel_speeds(left_speed, right_speed):
    left_speed = clamp(left_speed, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S)
    right_speed = clamp(right_speed, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S)
    fl_motor.setVelocity(left_speed)
    rl_motor.setVelocity(left_speed)
    fr_motor.setVelocity(right_speed)
    rr_motor.setVelocity(right_speed)

def drive_twist(v, omega):
    """Command a body twist (m/s, rad/s); returns the realized (v, omega, wL, wR)."""
    v_cmd, omega_cmd = clamp_twist(v, omega)
    left, right = wheel_speeds_from_twist(v_cmd, omega_cmd)
    set_wheel_speeds(left, right)
    return v_cmd, omega_cmd, left, right

def stop_robot():
    set_wheel_speeds(0.0, 0.0)

def get_front_laser_min():
    if laser is None:
        return float('inf')
    try:
        ranges = laser.getRangeImage()
    except Exception:
        return float('inf')
    if not ranges:
        return float('inf')
    n = len(ranges)
    start = int(0.4 * n)
    end = int(0.6 * n)
    front = [r for r in ranges[start:end] if r > 0.0 and math.isfinite(r)]
    return min(front) if front else float('inf')

def read_sensor_snapshot():
    """Read every enabled sensor and return a readings dict for sensor_debug."""
    r = {}

    # Wheel encoders
    for key, sensor in [
        ("enc_fl", fl_wheel_sensor), ("enc_fr", fr_wheel_sensor),
        ("enc_rl", rl_wheel_sensor), ("enc_rr", rr_wheel_sensor),
    ]:
        try:
            r[key] = sensor.getValue() if sensor else None
        except Exception:
            r[key] = None

    # IMU — getValues() for accel/gyro/compass, getRollPitchYaw() for inertial_unit
    for key, sensor, method in [
        ("accel",   accelerometer, "getValues"),
        ("gyro",    gyro,          "getValues"),
        ("compass", compass,       "getValues"),
        ("imu_rpy", inertial_unit, "getRollPitchYaw"),
    ]:
        try:
            r[key] = tuple(getattr(sensor, method)()) if sensor else None
        except Exception:
            r[key] = None

    # Range sensors
    for key, sensor in [
        ("range_fl", fl_range), ("range_fr", fr_range),
        ("range_rl", rl_range), ("range_rr", rr_range),
    ]:
        try:
            r[key] = sensor.getValue() if sensor else None
        except Exception:
            r[key] = None

    # Laser
    if laser:
        try:
            data = laser.getRangeImage()
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
    if camera_rgb:
        try:
            w = camera_rgb.getWidth()
            h = camera_rgb.getHeight()
            img = camera_rgb.getImage()
            idx = 4 * (h // 2 * w + w // 2)
            b, g, rv = img[idx], img[idx + 1], img[idx + 2]
            r["cam_rgb"] = (w, h, (rv, g, b))
        except Exception:
            r["cam_rgb"] = None
    else:
        r["cam_rgb"] = None

    # Depth camera — try RangeFinder API; silently degrade if device type differs
    if camera_depth:
        try:
            w = camera_depth.getWidth()
            h = camera_depth.getHeight()
            depth_data = camera_depth.getRangeImage()
            ctr = depth_data[h // 2 * w + w // 2] if depth_data else None
            r["cam_depth"] = (w, h, ctr)
        except Exception:
            r["cam_depth"] = None
    else:
        r["cam_depth"] = None

    return r

# ---------------------- control params ----------------
TARGET_LIN_VEL = 0.2    # m/s, forward/backward target speed for teleop
TARGET_ANG_VEL = 0.5    # rad/s, in-place rotation target rate for teleop
FRONT_STOP_DIST = 0.25  # meters: if obstacle closer than this, prevent forward motion

step_count = 0

# Keep track of the last non-empty key so short presses or holds are handled
current_key = -1
# test mode timer (when >0 the robot will drive forward to check motors)
test_timer = 0

# Sensor debug state
sensor_log_enabled = False   # toggled by L key
snapshot_cooldown  = 0       # prevents I from firing every step while held
log_toggle_cooldown = 0      # prevents L from toggling every step while held

print(
    "Controller ready.  Keys: F/S/A/D drive | Space stop | T self-test | "
    "I sensor snapshot | L toggle sensor log"
)

# ---------------------- main loop ---------------------
while robot.step(timestep) != -1:
    step_count += 1

    # Read all pending key events this timestep; keep the last non--1 as current
    key = keyboard.getKey()
    while key != -1:
        # log every raw key event for debugging
        print(f"raw key event: {key}")
        current_key = key
        key = keyboard.getKey()

    # Default twist: hold position
    v_cmd = 0.0
    omega_cmd = 0.0

    # Simple safety: check front obstacle distance
    front_min = get_front_laser_min()

    # --- Sensor debug: I = full snapshot, L = toggle compact per-step log ---
    if current_key in (ord('I'), ord('i')) and snapshot_cooldown == 0:
        print(format_sensor_snapshot(read_sensor_snapshot()))
        snapshot_cooldown = 20
    if snapshot_cooldown > 0:
        snapshot_cooldown -= 1

    if current_key in (ord('L'), ord('l')) and log_toggle_cooldown == 0:
        sensor_log_enabled = not sensor_log_enabled
        print(f"Sensor logging {'ON' if sensor_log_enabled else 'OFF'}")
        log_toggle_cooldown = 20
    if log_toggle_cooldown > 0:
        log_toggle_cooldown -= 1

    # Map ASDF to body twists (F forward, S back, A yaw-left, D yaw-right)
    if current_key in (ord('F'), ord('f')):
        if front_min >= FRONT_STOP_DIST:
            v_cmd = TARGET_LIN_VEL
    elif current_key in (ord('S'), ord('s')):
        v_cmd = -TARGET_LIN_VEL
    elif current_key in (ord('A'), ord('a')):
        omega_cmd = TARGET_ANG_VEL
    elif current_key in (ord('D'), ord('d')):
        omega_cmd = -TARGET_ANG_VEL
    elif current_key == ord(' '):
        v_cmd, omega_cmd = 0.0, 0.0

    # If 't' is pressed, schedule a short forward self-test
    if current_key in (ord('T'), ord('t')) and test_timer == 0:
        test_timer = int(2000 / max(1, timestep))  # 2 seconds worth of steps
        print(f"Starting 2s motor test at v={TARGET_LIN_VEL:.2f} m/s to verify motors")

    # Self-test overrides the user twist while it is active
    if test_timer > 0:
        v_cmd, omega_cmd = TARGET_LIN_VEL, 0.0
        test_timer -= 1
        if test_timer == 0:
            v_cmd, omega_cmd = 0.0, 0.0
            print("Motor test complete; stopping motors")

    # Convert commanded twist into wheel angular velocities and apply.
    v_real, omega_real, left_speed, right_speed = drive_twist(v_cmd, omega_cmd)

    # Debug output to Webots console
    if step_count % 10 == 0:  # throttle logging
        active = current_key if current_key != -1 else 'None'
        try:
            active_repr = chr(active) if isinstance(active, int) and 32 <= active <= 126 else active
        except Exception:
            active_repr = active
        print(
            f"step={step_count} key={active_repr} front_min={front_min:.3f} "
            f"cmd v={v_real:+.2f} m/s omega={omega_real:+.2f} rad/s "
            f"-> wL={left_speed:+.2f} wR={right_speed:+.2f} rad/s"
        )
        if sensor_log_enabled:
            print(format_compact_sensors(read_sensor_snapshot()))
