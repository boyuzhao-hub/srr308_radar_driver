from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('radar_conti_srr308')

    config_file = LaunchConfiguration('config_file')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'radar.yaml'),
        description='Full path to the ROS2 parameters file to use for the radar node'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Alias for config_file; accepts a different YAML file path'
    )

    lifecycle_nodes = ['radar_can0','radar_can1']

    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                             default_value="False",
                                             description=str("Use sim time argument for whether to force it"))

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    log_level = LaunchConfiguration('log_level')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='radar_lifecycle_manager',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    radar_can0_node = Node(
        package='radar_conti_srr308',
        executable='radar_conti_srr308_composition',
        name='radar_can0',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[LaunchConfiguration('params_file')]
    )

    radar_can1_node = Node(
        package='radar_conti_srr308',
        executable='radar_conti_srr308_composition',
        name='radar_can1',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[LaunchConfiguration('params_file')]
    )

    ld = LaunchDescription()
    ld.add_action(log_level_arg)
    ld.add_action(config_file_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(radar_can0_node)
    ld.add_action(radar_can1_node)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
