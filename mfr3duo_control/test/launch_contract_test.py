"""Evaluate the installed launch's actual description and rate parameters."""
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

path = Path(get_package_share_directory('mfr3duo_control')) / 'launch/control.launch.py'
spec = importlib.util.spec_from_file_location('control_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
for rate in (500, 1000):
    context = LaunchContext()
    context.launch_configurations.update(controller_update_rate=str(rate), viewer_enabled='false')
    nodes = [action for action in module.generate_launch_description().entities if isinstance(action, Node)]
    parameters = [evaluate_parameters(context, node._Node__parameters) for node in nodes]
    robot = next(p for values in parameters for p in values if isinstance(p, dict) and 'robot_description' in p)
    manager = next(p for values in parameters for p in values if isinstance(p, dict) and 'update_rate' in p)
    root = ET.fromstring(robot['robot_description'])
    period = float(root.find("ros2_control/hardware/param[@name='control_period']").text)
    assert manager['update_rate'] == rate
    assert math.isclose(period, 1 / rate, rel_tol=0, abs_tol=1e-12)
    assert root.find("ros2_control/hardware/param[@name='viewer_enabled']").text == 'false'
    assert len(root.findall('ros2_control/joint')) == 21
    print(f'PASS launch rate={rate}, control_period={period}, one hardware block')
    assert len(root.findall('ros2_control')) == 1
