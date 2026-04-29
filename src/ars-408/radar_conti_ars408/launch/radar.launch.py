from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path
import os
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # SRR308 Change: to radar.yaml
    pkg_dir = get_package_share_directory('radar_conti_ars408')
    default_params_file = PathJoinSubstitution(
        [pkg_dir, 'config', 'radar.yaml']
    )
    #######################################################################

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'radar.yaml'),
        description='Full path to the ROS2 parameters file to use for the radar node'
    )
    lifecycle_nodes = ['radar_node']

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

    # SRR308 Change: to obtain tf2 inforamtion from tf_radar_launch.py
    tf_launch_file = PathJoinSubstitution(
        [pkg_dir, 'launch', 'tf_radar_launch.py']
    )
    include_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch_file)
    )
    #######################################################################

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

    radar_node = Node(
        package='radar_conti_ars408',
        executable='radar_conti_ars408_composition',
        name='radar_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[LaunchConfiguration('params_file')])

    ld = LaunchDescription()
    ld.add_action(log_level_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(radar_node)
    # SRR308 Change: to include tf_radar_launch.py
    ld.add_action(include_tf_launch)
    #########################################################################
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
