---
description: "Phase 1 — Foundations: workspace ROS2, robot model, Gazebo world, Docker"
---

Execute Phase 1 of the autonomous-explorer-ros2 project. This phase builds the complete foundation.

### Deliverables

1. **ROS2 workspace**: Create `package.xml` and `setup.py` with correct dependencies (rclpy, sensor_msgs, geometry_msgs, nav_msgs, tf2_ros).
2. **Robot model SDF**: Design `explorer_bot/model.sdf` with:
   - Differential drive base (2 wheels + caster)
   - 2D LiDAR sensor (360°, 12m range, 10Hz)
   - RGB camera (640x480, 30fps)
   - IMU sensor
   - Proper collision and inertial properties
3. **Gazebo world**: Create `warehouse.sdf` — a simple but realistic warehouse with walls, shelves, corridors, and open areas (~20m x 15m).
4. **Launch files**: `simulation.launch.py` that starts Gazebo with the world and spawns the robot.
5. **Dockerfile**: Multi-stage Docker build based on `ros:humble`. `docker compose up` must launch Gazebo with the robot visible.
6. **rviz2 config**: Basic rviz config showing the robot model, lidar scan, and camera feed.

### Success Criteria

After this phase, running `docker compose up` should:
- Start Gazebo with the warehouse world loaded
- Spawn the explorer_bot at origin
- Show lidar rays and camera feed active in rviz2
- Allow teleop via `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### Constraints

- Follow the project architecture defined in GEMINI.md exactly.
- Use Gazebo Ignition (Gz Sim), not classic Gazebo.
- All sensor topics must follow ROS2 naming conventions.

Start by creating the workspace structure, then build each deliverable in order. Verify compilation at each step.
