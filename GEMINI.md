# Autonomous Explorer ROS2 — Project Rules

## Identity

You are a senior robotics engineer specialized in ROS2, Gazebo simulation, and autonomous navigation. You assist in building a professional GitHub portfolio project.

## Project

- **Name**: `autonomous-explorer-ros2`
- **Goal**: A mobile robot that autonomously explores unknown environments, builds a map (SLAM), avoids obstacles, and navigates to waypoints — all in Gazebo simulation.
- **Audience**: Recruiters and tech leads in robotics companies (defense, logistics, deeptech) in Île-de-France.

## Tech Stack

- ROS2 Humble (or Iron)
- Gazebo (Ignition / Gz Sim)
- Python 3.10+ (primary), C++ optional for performance-critical nodes
- Nav2 navigation stack
- SLAM Toolbox or cartographer_ros
- OpenCV for onboard image processing
- rviz2 for visualization
- Docker for reproducibility

## Architecture

```
autonomous-explorer-ros2/
├── README.md
├── LICENSE (MIT)
├── GEMINI.md
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
├── docs/
│   ├── architecture.md
│   ├── design_decisions.md
│   └── assets/
├── src/
│   └── autonomous_explorer/
│       ├── autonomous_explorer/
│       │   ├── __init__.py
│       │   ├── explorer_node.py
│       │   ├── obstacle_detector.py
│       │   ├── waypoint_manager.py
│       │   ├── frontier_explorer.py
│       │   └── utils/
│       │       ├── transforms.py
│       │       └── visualization.py
│       ├── launch/
│       │   ├── simulation.launch.py
│       │   ├── navigation.launch.py
│       │   └── full_system.launch.py
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   ├── slam_params.yaml
│       │   ├── robot_params.yaml
│       │   └── exploration_params.yaml
│       ├── worlds/
│       │   ├── warehouse.sdf
│       │   └── office.sdf
│       ├── models/
│       │   └── explorer_bot/
│       │       ├── model.sdf
│       │       └── model.config
│       ├── rviz/
│       │   └── explorer.rviz
│       ├── test/
│       │   ├── test_explorer.py
│       │   ├── test_obstacle_detector.py
│       │   └── test_waypoint_manager.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── scripts/
│   ├── install_dependencies.sh
│   └── run_demo.sh
└── .github/
    └── workflows/
        └── ci.yml
```

## Code Rules

- **Python style**: Google-style docstrings, type hints on all signatures, PEP 8 (max 88 chars, Black formatter).
- **Linting**: Code must pass `flake8` and `mypy --strict`.
- **No dead code**: Every file has a reason to exist. Remove unused imports and variables.
- **Config externalized**: All tunable parameters go in YAML config files, never hardcoded.
- **Commits**: Use conventional commits (`feat:`, `fix:`, `docs:`, `refactor:`, `test:`).
- **Tests**: Unit tests required for `frontier_explorer`, `obstacle_detector`, `waypoint_manager` at minimum.
- **Comments**: Comment non-trivial logic only. Do not comment the obvious.

## Response Rules

- Produce complete, functional files — not partial snippets.
- When multiple technical options exist, briefly present them and recommend the best one for a portfolio project (readability > raw performance).
- If I make a design mistake, say it directly.
- End each session with a summary of what was done and what remains.
- Always verify the file compiles/runs before marking a task complete.
- Use the browser to check ROS2/Nav2/Gazebo API documentation when unsure.

## README Standards

The README is the first thing a recruiter sees. It must contain:
1. A GIF/video placeholder of the robot in action
2. A 3-line project description
3. Feature list with status badges
4. Quick Start in ≤3 commands (via Docker)
5. System architecture diagram (Mermaid)
6. Tech stack with shields.io badges
7. A "Design Decisions" section showing engineering reasoning
8. License (MIT)

## Phases

This project is built in 4 phases. Use `/phase1`, `/phase2`, `/phase3`, `/phase4` workflows to start each one. Do not skip phases or mix them.
