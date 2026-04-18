# System Architecture

This repository is built around **ROS 2 (Jazzy/Humble)** and **Gazebo Harmonic**, creating a fully decoupled autonomous stack where the robotic intelligence is agnostic to the physical simulation.

## 1. ROS 2 Computation Graph

The following Mermaid diagram outlines the primary active nodes in the system during exploration.

```mermaid
graph TD
    %% Gazebo Layer
    subgraph Simulation [Gazebo Harmonic]
        GZ[Gazebo Server]
    end

    %% Bridges
    subgraph ROS-GZ Bridge
        Bridge[ros_gz_bridge]
        TF_Relay[tf_static_republisher]
    end

    %% Sensor processing
    subgraph Perception
        SLAM[slam_toolbox]
        ObsDet[obstacle_detector]
    end

    %% Navigation
    subgraph Navigation Stack
        Nav2[Nav2 Server]
        GlobalPlanner[Navfn Planner]
        LocalPlanner[DWB Local Planner]
    end

    %% Decision
    subgraph Intelligence
        Frontier[frontier_explorer_node]
        Waypoint[waypoint_manager]
    end

    %% Data Flow
    GZ -- "clock, scan, camera, imu, tf" --> Bridge
    Bridge -- "/tf_static_bridge" --> TF_Relay
    TF_Relay -- "/tf_static" --> SLAM
    Bridge -- "/scan" --> SLAM
    Bridge -- "/scan, /camera" --> ObsDet
    
    SLAM -- "/map" --> Frontier
    SLAM -- "/map, /tf" --> Nav2
    ObsDet -- "/obstacles" --> Nav2
    
    Frontier -- "/navigate_to_pose" --> Nav2
    Waypoint -- "/navigate_to_pose" --> Nav2
    Nav2 -- "/cmd_vel" --> Bridge
    Bridge -- "cmd_vel" --> GZ
```

## 2. Component Breakdown

### Perception & Mapping
- **`slam_toolbox`**: Operates in asynchronous mapping mode. Listens to odometry and `/scan` to build a 2D occupancy grid (`/map`). Solves loops dynamically.
- **`obstacle_detector`**: Fuses 2D Laser Scan and Camera data (simulated boundary detection) to publish point clouds or discrete `MarkerArray`s representing obstacles that aren't cleanly visible in global mapping, acting as a dynamic override for local costmaps.

### Autonomy
- **`frontier_explorer_node`**: A custom implementation combining Computer Vision (Canny edge detection via OpenCV) on the `/map` grid to find contiguous "unknown" boundaries (frontiers). It scores these frontiers based on distance and information gain, clustering them to avoid micro-movements, and sends Action goals to Nav2.
- **`waypoint_manager`**: An alternative autonomy mode for patrolling known environments based on predefined YAML coordinates.

### Navigation
- **Nav2**: The heart of movement. Uses a `Global Costmap` (fed by the static `/map` from SLAM) and a `Local Costmap` (fed dynamically by `/scan` and our `obstacle_detector`). Employs DWB (Dynamic Window Approach) to smoothly avoid obstacles en route to the frontier.

## 3. TF Frame Tree

To bridge the gap between Gazebo's absolute coordinate system and ROS 2's hierarchical tracking, a specific TF tree architecture was chosen to ensure SLAM Toolbox can properly track internal sensors.

```mermaid
graph TD
    Map[map] --> Odom[odom]
    Odom --> Footprint[base_footprint]
    Footprint --> BaseLink[base_link]
    
    %% The critical static offset bridge
    BaseLink -- "Z=-0.1" --> ModelRoot[explorer_bot]
    
    %% Gazebo internal frames
    ModelRoot --> Chassis[explorer_bot/chassis]
    Chassis --> LidarLink[explorer_bot/lidar_link]
    Chassis --> CameraLink[explorer_bot/camera_link]
    LidarLink --> Lidar[explorer_bot/lidar_link/gpu_lidar]
    CameraLink --> Camera[explorer_bot/camera_link/camera]
```

### The QoS Mismatch Resolution
A specific architectural decision was implemented regarding `tf_static`. Gazebo's Harmonic `PosePublisher` emits static frames dynamically upon entity creation but bridges them using a **VOLATILE** QoS profile. Standard ROS 2 tools (`tf2_ros`, `slam_toolbox`) strictly require **TRANSIENT_LOCAL** for `/tf_static` to support late joiners. 

The bridging is resolved by mapping Gazebo's TF to `/tf_static_bridge` and using our custom `tf_static_republisher` node to forward the payloads with correct durability.
