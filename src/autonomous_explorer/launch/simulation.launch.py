import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_explorer = get_package_share_directory('autonomous_explorer')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='warehouse.sdf')
    
    world_path = PathJoinSubstitution([pkg_autonomous_explorer, 'worlds', world_file])
    rviz_config_path = PathJoinSubstitution([pkg_autonomous_explorer, 'rviz', 'explorer.rviz'])
    model_path = PathJoinSubstitution([pkg_autonomous_explorer, 'models', 'explorer_bot', 'model.sdf'])

    # Gazebo Sim Server and GUI
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', model_path,
                   '-name', 'explorer_bot',
                   '-allow_renaming', 'true',
                   '-x', '-7.0',
                   '-y', '0.0',
                   '-z', '0.2']
    )

    # Bridge
    # cmd_vel (ROS -> GZ)
    # odom, tf (GZ -> ROS)
    # scan (GZ -> ROS)
    # camera/image_raw (GZ -> ROS)
    # imu (GZ -> ROS)
    bridge_params = os.path.join(pkg_autonomous_explorer, 'config', 'bridge.yaml')
    
    # Inline configuration for the bridge because we don't have bridge.yaml and want to keep it simple here.
    # We can use node parameters or command line arguments.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
        }],
        remappings=[
            ('/world/warehouse/model/explorer_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Gazebo bridge does not publish static TF from SDF models automatically.
    # Therefore, we provide static transform publishers for the model's links.
    tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link']
    )

    tf_base_to_model = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_model',
        output='screen',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'explorer_bot']
    )

    tf_static_relay = Node(
        package='autonomous_explorer',
        executable='tf_static_republisher',
        name='tf_static_republisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('world', default_value='warehouse.sdf', description='World file'),
        
        # Add install directory to Gazebo resource paths
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_autonomous_explorer, '..')
        ),

        gz_sim,
        spawn_entity,
        bridge,
        tf_base_link,
        tf_base_to_model,
        tf_static_relay,
        rviz
    ])
