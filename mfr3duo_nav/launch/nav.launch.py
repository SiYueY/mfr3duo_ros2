"""Static-map omnidirectional navigation using the shared hardware instance."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import SetRemap, Node


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
        DeclareLaunchArgument('localization', default_value='amcl', choices=['amcl', 'simulation_ground_truth']),
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
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'amcl'"])),
                launch_arguments={**parameters, 'map': LaunchConfiguration('map')}.items()),
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'simulation_ground_truth'"])),
                 parameters=[{'use_sim_time': False, 'yaml_filename': LaunchConfiguration('map')}]),
            Node(package='mfr3duo_nav', executable='simulation_localization.py', output='screen',
                 condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'simulation_ground_truth'"])),
                 parameters=[{'use_sim_time': False}]),
            TimerAction(period=2.0, actions=[
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
                     condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'simulation_ground_truth'"])),
                     parameters=[{'use_sim_time': False, 'autostart': True, 'bond_timeout': 0.0,
                                  'node_names': ['map_server', 'simulation_localization']}]),
            ]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
                launch_arguments=parameters.items()),
        ]),
    ])
