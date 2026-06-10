"""Launch file for dual pose graph SLAM node."""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_folder = get_package_share_directory('dual_pose_graph')
    config_file = os.path.join(package_folder, 'config', 'config.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace', description='Drone namespace', default_value='drone')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', description='Use simulation clock', default_value='false')
    config_file_arg = DeclareLaunchArgumentsFromConfigFile(
        'config_file', config_file, description='Configuration file')

    node = Node(
        package='dual_pose_graph',
        namespace=LaunchConfiguration('namespace'),
        executable='dual_pose_graph_node',
        name='dual_pose_graph_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfigurationFromConfigFile('config_file', default_file=config_file),
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        config_file_arg,
        node,
    ])
