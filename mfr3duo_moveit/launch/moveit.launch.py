"""MoveIt planning/execution with one optional shared control bringup."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    single = LaunchConfiguration('run_probe').perform(context).lower() == 'true'
    joint = LaunchConfiguration('run_joint_probe').perform(context).lower() == 'true'
    cancel = LaunchConfiguration('run_cancel_probe').perform(context).lower() == 'true'
    facade = LaunchConfiguration('run_facade_probe').perform(context).lower() == 'true'
    action_probe = LaunchConfiguration('run_action_probe').perform(context).lower() == 'true'
    if sum((single, joint, cancel, facade, action_probe)) > 1:
        raise RuntimeError('Only one POC may run at a time.')
    if (cancel or action_probe) and LaunchConfiguration('start_control').perform(context).lower() != 'false':
        raise RuntimeError('Cancellation fixture requires start_control:=false.')
    share = Path(get_package_share_directory('mfr3duo_moveit'))
    hardware = Path(get_package_share_directory('mfr3duo_hardware'))
    control = Path(get_package_share_directory('mfr3duo_control'))
    description = Command([FindExecutable(name='xacro'), ' ', str(hardware / 'ros2_control/mfr3duo.ros2_control.xacro'),
                           ' control_period:=', PythonExpression(['1.0 / ', LaunchConfiguration('controller_update_rate')]),
                           ' viewer_enabled:=', LaunchConfiguration('viewer_enabled')])
    def load(name):
        return yaml.safe_load((share / 'config' / name).read_text())
    parameters = [
        {'robot_description': ParameterValue(description, value_type=str),
         'robot_description_semantic': (share / 'config/mfr3duo.srdf').read_text(),
         'robot_description_kinematics': load('kinematics.yaml'),
         'robot_description_planning': load('joint_limits.yaml'),
         'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
         'ompl': load('ompl_planning.yaml'),
         'use_sim_time': False, 'publish_robot_description_semantic': True,
         'capabilities': '',
         # The software action fixture alone provides its own parent server.
         'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction' if action_probe else '',
         'allow_trajectory_execution': True,
         'publish_robot_description': False,
         'planning_scene_monitor_options': {'name': 'planning_scene_monitor',
             'robot_description': 'robot_description', 'joint_state_topic': '/joint_states',
             'attached_collision_object_topic': '/attached_collision_object',
             'publish_planning_scene_topic': '/publish_planning_scene',
             'monitored_planning_scene_topic': '/monitored_planning_scene'},
         'publish_planning_scene': True, 'publish_geometry_updates': True,
         'publish_state_updates': True, 'publish_transforms_updates': True},
        load('moveit_controllers.yaml'), load('facade.yaml')]
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(control / 'launch/control.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_control')),
        launch_arguments={'viewer_enabled': LaunchConfiguration('viewer_enabled'),
                          'controller_update_rate': LaunchConfiguration('controller_update_rate')}.items()),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=parameters, output='screen')]
    if LaunchConfiguration('run_probe').perform(context).lower() == 'true':
        actions.append(Node(package='mfr3duo_moveit', executable='planning_probe', parameters=parameters,
                            output='screen'))
    if LaunchConfiguration('run_joint_probe').perform(context).lower() == 'true':
        actions.append(Node(package='mfr3duo_moveit', executable='joint_planning_probe',
            parameters=parameters + [{'gate2_manifest': str(share / 'config/gate2_manifest.yaml'),
                                      'joint_probe_mode': LaunchConfiguration('joint_probe_mode')}], output='screen'))
    if action_probe:
        actions.append(Node(package='mfr3duo_moveit', executable='facade_action_probe', parameters=parameters, output='screen'))
    if facade:
        actions.append(Node(package='mfr3duo_moveit', executable='facade_runtime_probe', output='screen'))
    if cancel:
        actions.append(Node(package='mfr3duo_moveit', executable='official_stop_probe', parameters=parameters, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('viewer_enabled', default_value='true'),
        DeclareLaunchArgument('controller_update_rate', default_value='500'),
        DeclareLaunchArgument('run_probe', default_value='false'),
        DeclareLaunchArgument('run_joint_probe', default_value='false'),
        DeclareLaunchArgument('run_action_probe', default_value='false'),
        DeclareLaunchArgument('run_facade_probe', default_value='false'),
        DeclareLaunchArgument('run_cancel_probe', default_value='false'),
        DeclareLaunchArgument('joint_probe_mode', default_value='benchmark'),
        OpaqueFunction(function=setup)])
