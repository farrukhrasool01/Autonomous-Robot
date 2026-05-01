# CLAUDE.md — RosBot Webots Navigation Assignment

## Project Goal

Implement autonomous navigation for a RosBot in Webots using Python.

The robot must:
1. Start the simulation.
2. Reach the blue pillar first.
3. Then reach the yellow pillar.
4. Minimize simulation time.
5. Never drive over green ground.
6. Work in different maze worlds, not only one fixed world.

## Hard Constraints

Claude must never suggest or implement:
- Use of the Webots Supervisor API.
- Modifying the robot.
- Modifying the environment/world.
- Hardcoded maze-specific paths.
- Direct access to hidden global object positions.
- Solutions that only work in one maze layout.

## Available Robot Devices

Wheel motors/sensors:
- fl_wheel_joint
- fr_wheel_joint
- rl_wheel_joint
- rr_wheel_joint

IMU:
- imu accelerometer
- imu gyro
- imu compass
- imu inertial_unit

Range sensors:
- fl_range
- fr_range
- rl_range
- rr_range

Vision:
- camera rgb
- camera depth

Laser:
- laser

## Course/Syllabus-Aligned Robotics Approach

The solution should progressively use concepts from the Autonomous Robots syllabus provided in the file @Autonomous-Robots.pdf availble in the project folder:

1. Mobile robot kinematics
   - Convert desired linear/angular velocity into wheel speeds.

2. Localization
   - Estimate robot pose using wheel sensors and IMU.

3. Mapping
   - Use laser/depth/range data to build an occupancy grid.

4. Semantic perception
   - Use RGB/depth camera to detect blue pillar, yellow pillar, and green forbidden ground.

5. Frontier-based exploration
   - Explore unknown maze areas when the target is not visible.

6. Path planning
   - Use A* or Dijkstra on an inflated occupancy grid.

7. Path following control
   - Follow waypoints using a stable controller.

8. Safety layer
   - Use laser/range sensors to avoid collisions and avoid green ground.

## Development Method

This project must follow Spec-Driven Development.

Claude must follow this sequence:

1. Requirements
2. Design
3. Tasks
4. Implementation
5. Validation

Claude must not implement a feature until its requirements, design, and tasks have been created and approved.

## Coding Rules

- Use Python.
- Keep code modular.
- Prefer small, testable functions.
- Do not create one huge controller file unless necessary.
- If the code size increases create separate files.
- Add comments for robotics/math logic.
- Avoid unnecessary complexity in early steps.
- Each implementation step must be testable in Webots.

## Progressive Development Strategy

Do not build the final solution all at once.

Implement in milestones:

1. Basic controller setup and device initialization.
2. Wheel motion and stop behavior.
3. Sensor reading and debug output.
4. Basic obstacle avoidance.
5. RGB color detection for blue/yellow/green.
6. Depth-assisted target distance estimation.
7. Odometry and heading estimation.
8. Occupancy grid mapping.
9. A* path planning.
10. Frontier exploration.
11. Mission state machine.
12. Optimization for shortest simulation time.

## Response Style

When asked to work on this project, Claude should:
- First inspect relevant files.
- Explain what it understood.
- Propose a small next step.
- Ask for approval before implementation.
- After implementation, explain how to test it in Webots.