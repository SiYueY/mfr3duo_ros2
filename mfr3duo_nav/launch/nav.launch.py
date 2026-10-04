"""Static-map omnidirectional navigation using the shared hardware instance."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    share = Path(get_package_share_directory('mfr3duo_nav'))
    description = Path(get_package_share_directory('mfr3duo_description'))
    control = Path(get_package_share_directory('mfr3duo_control'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    parameters = {'use_sim_time': 'false', 'autostart': 'true',
                  'use_composition': 'False', 'params_file': LaunchConfiguration('params_file')}
    return LaunchDescription([
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('viewer_enabled', default_value='true'),
        DeclareLaunchArgument('controller_update_rate', default_value='500'),
        DeclareLaunchArgument('model_path', default_value=str(description / 'mjcf/navigation.xml')),
        DeclareLaunchArgument('map', default_value=str(share / 'maps/navigation.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/nav2.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / 'launch/control.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_control')),
            launch_arguments={key: LaunchConfiguration(key) for key in
                              ('viewer_enabled', 'controller_update_rate', 'model_path')}.items()),
        GroupAction([
            # Humble navigation_launch connects controller -> velocity_smoother.
            # Only the smoother's final output reaches the TMR controller.
            SetRemap(src='cmd_vel_smoothed', dst='/tmr_controller/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2 / 'launch/localization_launch.py')),
                launch_arguments={**parameters, 'map': LaunchConfiguration('map')}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
                launch_arguments=parameters.items()),
        ]),
    ])
