import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_autonomous_explorer = get_package_share_directory("autonomous_explorer")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")

    # Launch Parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Paths to config files
    slam_params_file = PathJoinSubstitution(
        [pkg_autonomous_explorer, "config", "slam_params.yaml"]
    )
    nav2_params_file = PathJoinSubstitution(
        [pkg_autonomous_explorer, "config", "nav2_params.yaml"]
    )

    # Start SLAM Toolbox (online async mode)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # Start Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
    )

    obstacle_detector_params_file = PathJoinSubstitution(
        [pkg_autonomous_explorer, "config", "obstacle_detector_params.yaml"]
    )

    # Start the custom obstacle_detector node
    obstacle_detector = Node(
        package="autonomous_explorer",
        executable="obstacle_detector",
        name="obstacle_detector",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, obstacle_detector_params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time (should be true for Gazebo)",
            ),
            slam_toolbox,
            nav2,
            obstacle_detector,
        ]
    )
