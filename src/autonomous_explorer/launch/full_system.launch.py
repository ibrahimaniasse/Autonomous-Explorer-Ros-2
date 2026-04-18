"""full_system.launch.py — One-command launch for the entire exploration stack.

Composes:
  - Phase 1: simulation.launch.py  (Gazebo + robot + bridge + rviz)
  - Phase 2: navigation.launch.py  (SLAM Toolbox + Nav2 + obstacle_detector)
  - Phase 3: frontier_explorer + waypoint_manager

Usage:
    ros2 launch autonomous_explorer full_system.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_autonomous_explorer = get_package_share_directory("autonomous_explorer")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Paths
    exploration_params_file = PathJoinSubstitution(
        [pkg_autonomous_explorer, "config", "exploration_params.yaml"]
    )

    # ------------------------------------------------------------------
    # Phase 1 — Simulation (Gazebo + robot + bridge + rviz)
    # ------------------------------------------------------------------
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autonomous_explorer, "launch", "simulation.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ------------------------------------------------------------------
    # Phase 2 — Navigation (SLAM + Nav2 + obstacle_detector)
    # ------------------------------------------------------------------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autonomous_explorer, "launch", "navigation.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ------------------------------------------------------------------
    # Phase 3 — Exploration
    # ------------------------------------------------------------------
    frontier_explorer = Node(
        package="autonomous_explorer",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            exploration_params_file,
        ],
    )

    waypoint_manager = Node(
        package="autonomous_explorer",
        executable="waypoint_manager",
        name="waypoint_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock.",
            ),
            simulation,
            navigation,
            frontier_explorer,
            waypoint_manager,
        ]
    )
