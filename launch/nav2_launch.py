import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,

            # SLAM enabled: slam_toolbox publishes map→odom live map
            'slam': 'True',

            # Localization DISABLED: AMCL not used. SLAM alone provides map frame.
            # No pre-built map or map_server needed; robot starts in unknown environment.
            'use_localization': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', description='Path to nav2 params yaml'),
        nav2_bringup,
    ])