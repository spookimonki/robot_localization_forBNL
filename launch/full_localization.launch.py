import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    robot_localization_dir = get_package_share_directory('robot_localization')

    mode = LaunchConfiguration('mode')
    real_condition = IfCondition(PythonExpression(["'", mode, "' == 'real'"]))

    autostart = DeclareLaunchArgument('autostart', default_value='true')
    filtered_odom_topic = DeclareLaunchArgument(
        'filtered_odom_topic',
        default_value='/odom/calibrated',
    )
    imu_x = DeclareLaunchArgument('imu_x', default_value='0.0')
    imu_y = DeclareLaunchArgument('imu_y', default_value='0.0')
    imu_z = DeclareLaunchArgument('imu_z', default_value='0.0')
    imu_roll = DeclareLaunchArgument('imu_roll', default_value='0.0')
    imu_pitch = DeclareLaunchArgument('imu_pitch', default_value='0.0')
    imu_yaw = DeclareLaunchArgument('imu_yaw', default_value='0.0')

    # Hardware nodes - only run in real mode
    wheel_odom_node = Node(
        package='robot_localization',
        executable='wheel_odom_node',
        name='wheel_odom_node',
        output='screen',
        condition=real_condition,
    )

    imu_node = Node(
        package='robot_localization',
        executable='imu_odom_node',
        name='imu_odom_node',
        output='screen',
        condition=real_condition,
    )

    lidar_node = Node(
        package='robot_localization',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        condition=real_condition,
    )

    static_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_static_tf',
        output='screen',
        arguments=[
            LaunchConfiguration('imu_x'),
            LaunchConfiguration('imu_y'),
            LaunchConfiguration('imu_z'),
            LaunchConfiguration('imu_yaw'),
            LaunchConfiguration('imu_pitch'),
            LaunchConfiguration('imu_roll'),
            'base_link',
            'imu_link',
        ],
    )

    ukf_node = LifecycleNode(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        namespace='',
        output='screen',
        autostart=LaunchConfiguration('autostart'),  # type: ignore[arg-type]
        parameters=[
            os.path.join(robot_localization_dir, 'params', 'ukf.yaml'),
            {'publish_tf': False},
        ],
        remappings=[
            ('accel/filtered', 'acceleration/filtered'),
            ('odometry/filtered', LaunchConfiguration('filtered_odom_topic')),
        ],
    )

    return LaunchDescription([
        autostart,
        filtered_odom_topic,
        imu_x,
        imu_y,
        imu_z,
        imu_roll,
        imu_pitch,
        imu_yaw,
        wheel_odom_node,
        imu_node,
        lidar_node,
        static_base_to_imu,
        ukf_node,
    ])
