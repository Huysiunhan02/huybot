import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

pkg_huybot_bringup = get_package_share_directory('huybot_bringup')
pkg_huybot_control = get_package_share_directory('huybot_control')
pkg_huybot_description = get_package_share_directory('huybot_description')

def generate_launch_description():

    # Declares launch arguments
    camera_arg = DeclareLaunchArgument(
            'include_camera',
            default_value='False',
            description='Indicates whether to include camera launch.')
    camera =  LaunchConfiguration('include_camera')
    rplidar_arg = DeclareLaunchArgument(
            'include_rplidar',
            default_value='True',
            description='Indicates whether to include rplidar launch.')
    rplidar =  LaunchConfiguration('include_rplidar')

    # Includes huybot_description launch file
    include_huybot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_huybot_description, 'launch', 'huybot_description.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true',
        }.items()
    )

    # Include huybot_control launch file
    include_huybot_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_huybot_control, 'launch', 'huybot_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    # Include rplidar launch file
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_huybot_bringup, 'launch', 'rplidar.launch.py'),
        ),
        launch_arguments={
            "serial_port": '/dev/ttyUSB_LIDAR',
        }.items(),
                condition=IfCondition(rplidar)
    )
    # Include camera launch file
    include_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_huybot_bringup, 'launch', 'camera.launch.py'),
        ),
        launch_arguments={
        }.items(),
                condition=IfCondition(camera)
    )

    # Waits for huybot_description to set up robot_state_publisher.
    huybot_control_timer = TimerAction(period=5.0, actions=[include_huybot_control])
    # Defer sensors launch to avoid overhead while robot_state_publisher is setting up.
    rplidar_timer = TimerAction(period=3.0, actions=[include_rplidar])
    camera_timer = TimerAction(period=3.0, actions=[include_camera])

    # Launch them all!
    return LaunchDescription([
        include_huybot_description,
        huybot_control_timer,
        camera_arg,
        camera_timer,
        rplidar_arg,
        rplidar_timer,
    ])