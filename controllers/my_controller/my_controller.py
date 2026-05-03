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
from reactive import laser_5_sectors, wall_follow_twist
# local_plan and candidate functions remain in reactive.py for reference but are not called here

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

def read_depth_front_min():
    """5th-percentile depth in the front-centre ROI for floating-obstacle detection.

    ROI: centre third of width, upper two-thirds of height, sampled every 2 px.
    Returns float('inf') when depth is unavailable or the ROI has no valid pixels.
    """
    if camera_depth is None:
        return float('inf')
    try:
        w = camera_depth.getWidth()
        h = camera_depth.getHeight()
        data = camera_depth.getRangeImage()
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
    idx = max(0, len(vals) // 20)   # 5th percentile
    return vals[idx]

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
# TARGET_LIN_VEL = 0.16    # m/s, forward/backward target speed for teleop
# TARGET_ANG_VEL = 0.2    # rad/s, in-place rotation target rate for teleop
# FRONT_STOP_DIST = 0.10  # meters: hard-stop threshold (laser center zone)
# # Wall-following constants (M5 — active autonomous behavior)
# FRONT_BLOCK_DIST = 0.30  # front clearance below this triggers rotate-left (m)
# WALL_TARGET_DIST = 0.35  # desired right-wall following distance (m)
# WALL_CLOSE_BAND  = 0.12  # dead-band half-width around wall target (m)
# WALL_LOST_DIST   = 0.70  # right distance above this → wall lost, seek right (m)
# SIDE_DANGER_DIST = 0.18  # side clearance below this → danger steer (m)
# OMEGA_SMALL      = 0.22  # gentle wall-correction angular rate (rad/s)
# BLOCK_TIMEOUT    = 40    # front-blocked steps before timeout recovery activates
# REAR_SAFE_DIST   = 0.25  # rear range below this → do not reverse (m)

TARGET_LIN_VEL = 0.05
TARGET_ANG_VEL = 0.50


FRONT_CAUTION_DIST = 0.35
CAUTION_LIN_VEL = 0.04
CAUTION_ANG_VEL = 0.08


FRONT_STOP_DIST  = 0.15
FRONT_BLOCK_DIST = 0.25

WALL_TARGET_DIST = 0.35
WALL_CLOSE_BAND  = 0.12
WALL_LOST_DIST   = 0.80

SIDE_DANGER_DIST = 0.05
OMEGA_SMALL      = 0.22

BLOCK_TIMEOUT  = 40
REAR_SAFE_DIST = 0.25

# Depth camera safety constants (M5.1 — floating obstacle detection)
DEPTH_CAUTION_DIST     = 0.60  # slow down below this
DEPTH_FRONT_BLOCK_DIST = 0.45  # hard block below this
DEPTH_FRONT_STOP_DIST  = 0.10  # emergency below this
DEPTH_MIN_VALID        = 0.10


step_count = 0

# Keyboard edge detection
previous_pressed_keys = set()

# test mode timer (when >0 the robot will drive forward to check motors)
test_timer = 0

# Sensor debug state
sensor_log_enabled = False   # toggled by L key

# Autonomous mode state
auto_mode   = False   # toggled by G key
block_timer = 0       # consecutive steps with front clearance < FRONT_BLOCK_DIST

print(
    "Controller ready.  Keys: F/S/A/D drive | Space stop | T self-test | "
    "G autonomous mode | I sensor snapshot | L toggle sensor log"
)

# ---------------------- main loop ---------------------
while robot.step(timestep) != -1:
    step_count += 1

    # Build key sets for edge detection this timestep
    pressed_keys_now = set()
    k = keyboard.getKey()
    while k != -1:
        pressed_keys_now.add(k)
        k = keyboard.getKey()
    # new_key_events: keys that just appeared (not held from last step)
    new_key_events = pressed_keys_now - previous_pressed_keys

    # Default twist: hold position
    v_cmd = 0.0
    omega_cmd = 0.0
    sel_label = ''

    # Simple safety: check front obstacle distance
    front_min = get_front_laser_min()

    # --- Sensor debug: I = snapshot (one-shot), L = toggle log (one-shot) ---
    if ord('I') in new_key_events or ord('i') in new_key_events:
        print(format_sensor_snapshot(read_sensor_snapshot()))

    if ord('L') in new_key_events or ord('l') in new_key_events:
        sensor_log_enabled = not sensor_log_enabled
        print(f"Sensor logging {'ON' if sensor_log_enabled else 'OFF'}")

    # --- Autonomous mode toggle: G (one-shot, exactly once per physical press) ---
    if ord('G') in new_key_events or ord('g') in new_key_events:
        auto_mode = not auto_mode
        if not auto_mode:
            block_timer = 0
        print(f"Autonomous mode {'ON' if auto_mode else 'OFF'}")

    # --- Motion: Space is a hard stop (one-shot) that also exits autonomous mode ---
    if ord(' ') in new_key_events:
        v_cmd, omega_cmd = 0.0, 0.0
        if auto_mode:
            auto_mode = False
            block_timer = 0
            print("Autonomous mode OFF (Space pressed)")

    # --- Autonomous: right-hand wall following (M5 — active behavior) ---
    # local_plan() and candidate planner remain in reactive.py but are not called here.
    elif auto_mode:
        raw_ranges = []
        if laser:
            try:
                raw_ranges = laser.getRangeImage() or []
            except Exception:
                raw_ranges = []
        sectors = laser_5_sectors(raw_ranges)
        _, left_min, center_min, right_min, _ = sectors

        fl_val = fl_range.getValue() if fl_range else float('inf')
        fr_val = fr_range.getValue() if fr_range else float('inf')
        rl_val = rl_range.getValue() if rl_range else float('inf')
        rr_val = rr_range.getValue() if rr_range else float('inf')

        rear_safe = rl_val >= REAR_SAFE_DIST and rr_val >= REAR_SAFE_DIST



        depth_front_min = read_depth_front_min()
        depth_valid = math.isfinite(depth_front_min)

        depth_caution = (
            depth_valid and depth_front_min < DEPTH_CAUTION_DIST
        )

        depth_blocked = (
            depth_valid and depth_front_min < DEPTH_FRONT_BLOCK_DIST
        )

        depth_emg = (
            depth_valid and depth_front_min < DEPTH_FRONT_STOP_DIST
        )

        laser_blocked = center_min < FRONT_BLOCK_DIST

        # IMPORTANT:
        # Only real blocks increment block_timer.
        # Depth caution must NOT increment block_timer.
        if laser_blocked or depth_blocked:
            block_timer += 1
        else:
            block_timer = 0

        # If depth is truly blocked, bypass wall following and turn away.
        if laser_blocked or depth_blocked:
            v_cmd = 0.0
            omega_cmd = TARGET_ANG_VEL
            sel_label = "depth_block_turn_left"

        else:
            # In caution zone, still allow wall-following.
            # Do NOT use depth as fused front unless it is actually blocked.
            fused_front = center_min

            v_cmd, omega_cmd, sel_label = wall_follow_twist(
                    fused_front, right_min, left_min,
                    block_timer, rear_safe,
                    FRONT_BLOCK_DIST, FRONT_CAUTION_DIST,
                    WALL_TARGET_DIST, WALL_CLOSE_BAND, WALL_LOST_DIST, SIDE_DANGER_DIST,
                    BLOCK_TIMEOUT,
                    TARGET_LIN_VEL, CAUTION_LIN_VEL,
                    OMEGA_SMALL, CAUTION_ANG_VEL, TARGET_ANG_VEL
                )

        # Caution means pass slowly, not recovery.
            if depth_caution and v_cmd > 0:
                v_cmd = min(v_cmd, 0.07)
                sel_label = sel_label + "_depth_caution"

                
                # Depth camera front obstacle detection (M5.1 — catches floating slabs)
        depth_front_min = read_depth_front_min()
        laser_blocked   = center_min < FRONT_BLOCK_DIST
        
        # Hard emergency override — laser/range sensors (unchanged from M5)
        laser_front_emg = center_min < FRONT_STOP_DIST
        fl_emg = fl_val < FRONT_STOP_DIST
        fr_emg = fr_val < FRONT_STOP_DIST
        front_emg = laser_front_emg or fl_emg or fr_emg

        # Forward emergency: stop forward motion, but rotate to escape.
        if v_cmd > 0 and front_emg:
            v_cmd = 0.0

            if fl_emg and not fr_emg:
                omega_cmd = -TARGET_ANG_VEL
                sel_label = "front_left_emg_turn_right"
            elif fr_emg and not fl_emg:
                omega_cmd = TARGET_ANG_VEL
                sel_label = "front_right_emg_turn_left"
            else:
                omega_cmd = TARGET_ANG_VEL
                sel_label = "front_emg_turn_left"

        # Depth emergency: also rotate, do not freeze.
        if v_cmd > 0 and depth_emg:
            v_cmd = 0.0
            omega_cmd = TARGET_ANG_VEL
            sel_label = "depth_emg_turn_left"

        # Rear safety: rear sensors should only block reverse motion.
        rear_blocked = rl_val < REAR_SAFE_DIST or rr_val < REAR_SAFE_DIST

        if v_cmd < 0 and rear_blocked:
            v_cmd = 0.0
            omega_cmd = TARGET_ANG_VEL
            sel_label = "rear_blocked_turn_left"
            
        if step_count % 10 == 0:
            print(
                f"[DEPTH] front={depth_front_min:.3f} "
                f"caution={depth_caution} block={depth_blocked} emg={depth_emg} "
                f"block_timer={block_timer} | {sel_label} "
                f"v={v_cmd:+.2f} omega={omega_cmd:+.2f}"
            )


    # --- Teleop: ASDF keyboard motion — continuous while key held ---
    else:
        if ord('F') in pressed_keys_now or ord('f') in pressed_keys_now:
            if front_min >= FRONT_STOP_DIST:
                v_cmd = TARGET_LIN_VEL
        elif ord('S') in pressed_keys_now or ord('s') in pressed_keys_now:
            v_cmd = -TARGET_LIN_VEL
        elif ord('A') in pressed_keys_now or ord('a') in pressed_keys_now:
            omega_cmd = TARGET_ANG_VEL
        elif ord('D') in pressed_keys_now or ord('d') in pressed_keys_now:
            omega_cmd = -TARGET_ANG_VEL

    # --- T self-test: one-shot, disables autonomous mode ---
    if (ord('T') in new_key_events or ord('t') in new_key_events) and test_timer == 0:
        if auto_mode:
            auto_mode = False
            block_timer = 0
            print("Autonomous mode OFF (motor self-test starting)")
        test_timer = int(2000 / max(1, timestep))  # 2 seconds worth of steps
        print(f"Starting 2s motor test at v={TARGET_LIN_VEL:.2f} m/s to verify motors")

    # Self-test overrides the user twist while it is active
    if test_timer > 0:
        v_cmd, omega_cmd = TARGET_LIN_VEL, 0.0
        test_timer -= 1
        if test_timer == 0:
            v_cmd, omega_cmd = 0.0, 0.0
            print("Motor test complete; stopping motors")

    # Advance edge-detection state before next timestep
    previous_pressed_keys = pressed_keys_now

    # Convert commanded twist into wheel angular velocities and apply.
    v_real, omega_real, left_speed, right_speed = drive_twist(v_cmd, omega_cmd)

    # Debug output to Webots console
    if step_count % 10 == 0:  # throttle logging
        mode_str = f"AUTO:{sel_label}" if auto_mode else "TELE"
        print(
            f"step={step_count} [{mode_str}] front_min={front_min:.3f} "
            f"cmd v={v_real:+.2f} m/s omega={omega_real:+.2f} rad/s "
            f"-> wL={left_speed:+.2f} wR={right_speed:+.2f} rad/s"
        )
        if sensor_log_enabled:
            print(format_compact_sensors(read_sensor_snapshot()))
