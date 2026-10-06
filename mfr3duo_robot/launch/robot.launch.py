"""Whole-robot bringup: one hardware, wall-time MoveIt/Nav2 and observation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def setup(context):
    package = lambda name: Path(get_package_share_directory(name))
    robot = package('mfr3duo_robot')
    scene = LaunchConfiguration('scene').perform(context)
    model = str(package('mfr3duo_description') / 'mjcf/manipulation.xml')
    keyframe, objects = 'task_home', 'box=grasp_object_box=grasp_object_box_collision'
    task_parameters = []
    scene_joints = ''
    localization = 'amcl'
    map_file = str(robot / 'maps/tasks.yaml')
    navigation = RewrittenYaml(
        source_file=str(package('mfr3duo_nav') / 'config/nav2.yaml'),
        root_key='', convert_types=True,
        param_rewrites={'amcl.ros__parameters.initial_pose.x': '-0.6',
                        'amcl.ros__parameters.initial_pose.y': '0.7'})
    if scene in ('kitchen', 'kitchen_interactive'):
        from mfr3duo_scenes.compose import compose_scene
        directory = package('mfr3duo_scenes') / 'scenes/kitchen'
        model, config = compose_scene(directory, package('mfr3duo_description'),
                                      interactive=scene == 'kitchen_interactive')
        model = str(model)
        keyframe, objects = config['initial_keyframe'], config['grasp_objects']
        scene_joints = config['scene_joints']
        localization = config['localization']
        navigation = str(directory / 'nav2.yaml')
        map_file = str(directory / 'maps/kitchen.yaml')
        task_parameters = [{'grasp_profile_path': str(directory / 'grasp.yaml'),
                            'navigation_posture_path': str(directory / 'navigation_posture.yaml'),
                            'environment_scene_path': str(directory / 'planning_scene.yaml'),
                            'robot.transit_velocity_scaling': 0.20,
                            'robot.transit_acceleration_scaling': 0.10,
                            'robot.gripper_open_width': 0.078,
                            'robot.grasp_spine_height': 0.25,
                            # Pick preflight plans the arm and spine as one collision-checked
                            # trajectory.  Execute the same trajectory so a feasible plan is
                            # a valid predictor of the physical pregrasp motion.
                            'robot.pregrasp_spine_first': False,
                            'execution.timeout_margin': 15.0,
                            'navigator.localization_node': ('simulation_localization'
                                if localization == 'simulation_ground_truth' else 'amcl'),
                            'demo_navigation_pose': config['waypoints']['aisle'],
                            'demo_place_pose': config['place_pose']}]
        if scene == 'kitchen_interactive':
            task_parameters[0]['scene_interactions_path'] = str(directory / 'interactions.yaml')
            task_parameters[0]['environment_scene_path'] = str(directory / 'planning_scene_interactive.yaml')
    elif scene != 'tasks':
        raise ValueError('Unknown scene; choose tasks, kitchen, or kitchen_interactive')
    include = lambda name, filename, arguments: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package(name) / 'launch' / filename)),
        launch_arguments=arguments.items())
    return [
        include('mfr3duo_control', 'control.launch.py', {
            'viewer_enabled': LaunchConfiguration('viewer_enabled'),
            'controller_update_rate': LaunchConfiguration('controller_update_rate'),
            'model_path': model,
            'initial_keyframe': keyframe,
            'scene_joints': scene_joints,
            'grasp_objects': objects}),
        include('mfr3duo_moveit', 'moveit.launch.py', {'start_control': 'false'}),
        include('mfr3duo_nav', 'nav.launch.py', {'start_control': 'false',
            'params_file': navigation, 'map': map_file, 'localization': localization}),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
                        '--frame-id','map','--child-frame-id','simulation_world']),
        Node(package='mfr3duo_robot', executable='task_demo',
             condition=IfCondition(LaunchConfiguration('run_demo')),
             parameters=[str(robot / 'config/robot.yaml'), *task_parameters], output='screen'),
        Node(package='mfr3duo_robot', executable='task_demo', name='robot_task_server',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('serve_tasks'), "' == 'true' and '",
                 LaunchConfiguration('run_demo'), "' != 'true'"])),
             parameters=[str(robot / 'config/robot.yaml'), *task_parameters,
                         {'serve_tasks': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='tasks'),
        DeclareLaunchArgument('viewer_enabled', default_value='true'),
        DeclareLaunchArgument('controller_update_rate', default_value='500'),
        DeclareLaunchArgument('run_demo', default_value='false'),
        DeclareLaunchArgument('serve_tasks', default_value='true'),
        OpaqueFunction(function=setup),
    ])
