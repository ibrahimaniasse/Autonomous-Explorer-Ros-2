# Design Decisions

Developing an autonomous exploration robot involves balancing processing efficiency, algorithmic completeness, and system resilience.

## SLAM Toolbox vs. Cartographer

**Problem:** We need a robust algorithm to generate 2D occupancy grids while the robot explores unknown spaces.
**Options:** Cartographer (Google), RTAB-Map, SLAM Toolbox (Steve Macenski).
**Choice:** **SLAM Toolbox**
**Rationale:** While Cartographer is extremely powerful for static map-building from rosbags, SLAM Toolbox is explicitly designed for lifelong mapping and real-time navigation integration. It operates asynchronously, dynamically expanding its internal grid, and integrates natively with Nav2. Furthermore, its configuration overhead is significantly smaller for simulated environments compared to Cartographer's complex lua scripts.

## Custom Frontier Exploration vs. Nav2 Explore / Explore-Lite

**Problem:** Navigating the robot to iteratively uncover all unknown space in an environment.
**Options:** Port `explore_lite` from ROS 1, use an existing potential field explorer, or build a custom Computer Vision-based ROS 2 node.
**Choice:** **Custom Frontier Explorer (`frontier_explorer_node`)**
**Rationale:** Writing a custom node using OpenCV Canny edge detection directly on the occupancy grid provides ultimate control over cluster filtering and recovery behaviors. Existing ports often suffer from "Stale Goal" deadlocks where they request Nav2 to move to unreachable points inside walls. Our custom implementation includes built-in safeguards, blacklisting of invalid Nav2 goals, and size-based clustering ensuring the robot ignores un-explorable artifacts.

## Docker-First Development Strategy

**Problem:** ROS 2 relies heavily on system-level dependencies (rclcpp, fastrtps, specific python bindings) mapped strictly to Ubuntu versions (Jazzy on 24.04).
**Options:** Require users to install Ubuntu natively via dual-boot, provide a heavy VM, or use Docker.
**Choice:** **Docker Compose + ROS Base Images**
**Rationale:** For portfolio presentation and team collaboration, the time-to-first-run is the most critical metric. By isolating the workspace in an `osrf/ros:jazzy-desktop` derived image with host networking and X11 forwarding, any reviewer (even using a Mac or Windows via WSL) can compile and launch the entire Gazebo simulation without polluting their host environment. 

## TF QoS Translation

**Problem:** Gazebo Harmonic's `PosePublisher` broadcasts static TF transforms with a `VOLATILE` QoS. ROS 2's `tf2_ros` explicitly requires `TRANSIENT_LOCAL` for `/tf_static` static transforms.
**Options:** Patch Gazebo upstream, configure `tf2_ros` to accept volatile messages, or build a relay.
**Choice:** **Custom `tf_static_republisher` node.**
**Rationale:** Altering global `tf2_ros` QoS would break ecosystem compatibility. Patching Gazebo is out of scope. A lightweight python relay simply shifting the durability profile provides a transparent architectural layer bridging the mismatch.

## High-Density LiDAR (1440 samples)

**Problem:** SLAM maps in large simulation worlds suffered from "Sieve" or "Spike" effects at long ranges, disabling edge detection.
**Options:** Implement morphological dilation in ROS, lower the lidar resolution, or increase simulation ray density.
**Choice:** **Gazebo `<samples>1440</samples>`**
**Rationale:** Bresenham line algorithms used in 2D raytracing create gaps when the angular distance between rays exceeds the grid resolution (0.05m). Simulating a higher-quality LiDAR (1440 samples = 0.25 deg) resolves the map fracturing natively without heavy image processing overheads, mirroring real-world SICK/Velodyne performance.
