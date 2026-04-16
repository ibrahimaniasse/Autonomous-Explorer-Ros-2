---
description: "Phase 2 — Perception & Mapping: SLAM, obstacle detection, Nav2 base config"
---

Execute Phase 2 of the autonomous-explorer-ros2 project. This phase adds perception and cartography.

### Prerequisites

Phase 1 must be complete. The robot must be spawnable in Gazebo with functional sensors.

### Deliverables

1. **SLAM integration**: Configure SLAM Toolbox (`slam_params.yaml`) for online async SLAM. The robot must build an occupancy grid map in real time while being teleoperated.
2. **obstacle_detector node**: Create `obstacle_detector.py` that:
   - Subscribes to `/scan` (LiDAR) and `/camera/image_raw` (RGB)
   - Detects obstacles within a configurable danger zone (default: 0.5m)
   - Publishes obstacle markers on `/obstacles` (visualization_msgs/MarkerArray)
   - Uses OpenCV for basic camera-based obstacle detection (edge detection or color segmentation)
   - Fuses LiDAR + camera detections with configurable weights
3. **Nav2 base configuration**: Set up `nav2_params.yaml` with:
   - Global costmap (from SLAM map)
   - Local costmap (from LiDAR, 3m radius)
   - DWB local planner
   - NavFn global planner
   - Recovery behaviors (spin, backup, wait)
4. **rviz2 visualization**: Update rviz config to display:
   - Occupancy grid map building in real time
   - Robot pose (TF tree)
   - Detected obstacles (markers)
   - Costmaps (global + local)
   - Camera feed panel
5. **Launch file**: Create `navigation.launch.py` that starts SLAM + Nav2 + obstacle_detector.

### Success Criteria

- Teleoperating the robot builds a coherent map visible in rviz2.
- Obstacles are detected and visualized as markers.
- Nav2 is running and accepting goals via rviz2 "Nav2 Goal" button.
- The robot navigates to a clicked goal while avoiding obstacles.

### Constraints

- SLAM Toolbox, not Cartographer (simpler setup, sufficient for portfolio).
- All parameters in YAML configs, nothing hardcoded.
- obstacle_detector must have Google-style docstrings and type hints.
- Write unit test `test_obstacle_detector.py` with at least 3 test cases.
