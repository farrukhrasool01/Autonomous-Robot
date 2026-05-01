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
MAX_WHEEL_SPEED = 10.0

def set_wheel_speeds(left_speed, right_speed):
    left_speed = clamp(left_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    right_speed = clamp(right_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    fl_motor.setVelocity(left_speed)
    rl_motor.setVelocity(left_speed)
    fr_motor.setVelocity(right_speed)
    rr_motor.setVelocity(right_speed)

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

# ---------------------- control params ----------------
BASE_SPEED = 3.0      # forward/backward speed
TURN_SPEED = 2.0      # in-place rotation speed
FRONT_STOP_DIST = 0.25  # meters: if obstacle closer than this, prevent forward motion

step_count = 0

# Keep track of the last non-empty key so short presses or holds are handled
current_key = -1
# test mode timer (when >0 the robot will drive forward to check motors)
test_timer = 0

print("Keyboard controller initialized. Click the 3D view so it has keyboard focus, then use ASDF keys (F/S/A/D) to drive. Press 't' to run a short motor test.")

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

    # Default: stop
    left_speed = 0.0
    right_speed = 0.0

    # Simple safety: check front obstacle distance
    front_min = get_front_laser_min()

    # Map ASDF to motions (F forward, S back, A left, D right)
    if current_key in (ord('F'), ord('f')):
        # forward unless obstacle too close
        if front_min < FRONT_STOP_DIST:
            left_speed = 0.0
            right_speed = 0.0
        else:
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED
    elif current_key in (ord('S'), ord('s')):
        left_speed = -BASE_SPEED
        right_speed = -BASE_SPEED
    elif current_key in (ord('A'), ord('a')):
        print("AAAAA")
        # rotate left in place
        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED
    elif current_key in (ord('D'), ord('d')):
        # rotate right in place
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED
    elif current_key == ord(' '):
        # spacebar explicitly stops
        left_speed = 0.0
        right_speed = 0.0
    else:
        # no relevant key: stop motors
        left_speed = 0.0
        right_speed = 0.0

    # Apply speeds
    set_wheel_speeds(left_speed, right_speed)

    # If 't' is pressed, run a short motor test regardless of laser
    if current_key in (ord('T'), ord('t')) and test_timer == 0:
        test_timer = int(2000 / max(1, timestep))  # 2 seconds worth of steps
        print("Starting 2s motor test at BASE_SPEED to verify motors")

    if test_timer > 0:
        # during test, override speeds
        set_wheel_speeds(BASE_SPEED, BASE_SPEED)
        test_timer -= 1
        if test_timer == 0:
            print("Motor test complete; stopping motors")
            set_wheel_speeds(0.0, 0.0)

    # Debug output to Webots console to help diagnose why motors may not move
    if step_count % 10 == 0:  # throttle logging
        active = current_key if current_key != -1 else 'None'
        # try to show a readable character when possible
        try:
            active_repr = chr(active) if isinstance(active, int) and 32 <= active <= 126 else active
        except Exception:
            active_repr = active
        print(f"step={step_count} active_key={active_repr} front_min={front_min:.3f} left_speed={left_speed} right_speed={right_speed}")
