"""Whole-robot bringup: one hardware, wall-time MoveIt/Nav2 and observation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package = lambda name: Path(get_package_share_directory(name))
    robot = package('mfr3duo_robot')
    navigation = RewrittenYaml(
        source_file=str(package('mfr3duo_nav') / 'config/nav2.yaml'),
        root_key='', convert_types=True,
        param_rewrites={'amcl.ros__parameters.initial_pose.x': '-0.6',
                        'amcl.ros__parameters.initial_pose.y': '0.7'})
    include = lambda name, filename, arguments: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package(name) / 'launch' / filename)),
        launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('viewer_enabled', default_value='true'),
        DeclareLaunchArgument('controller_update_rate', default_value='500'),
        DeclareLaunchArgument('run_demo', default_value='false'),
        include('mfr3duo_control', 'control.launch.py', {
            'viewer_enabled': LaunchConfiguration('viewer_enabled'),
            'controller_update_rate': LaunchConfiguration('controller_update_rate'),
            'model_path': str(package('mfr3duo_description') / 'mjcf/manipulation.xml'),
            'initial_keyframe': 'task_home',
            'grasp_objects': 'box=grasp_object_box=grasp_object_box_collision'}),
        include('mfr3duo_moveit', 'moveit.launch.py', {'start_control': 'false'}),
        include('mfr3duo_nav', 'nav.launch.py', {'start_control': 'false',
            'params_file': navigation, 'map': str(robot / 'maps/tasks.yaml')}),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
                        '--frame-id','map','--child-frame-id','simulation_world']),
        Node(package='mfr3duo_robot', executable='task_demo',
             condition=IfCondition(LaunchConfiguration('run_demo')),
             parameters=[str(robot / 'config/robot.yaml')], output='screen'),
    ])
