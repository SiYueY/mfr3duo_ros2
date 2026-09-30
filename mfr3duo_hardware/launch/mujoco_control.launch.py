"""Start the MuJoCo hardware, robot description, and standard controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("mfr3duo_hardware")
    xacro = PathJoinSubstitution([share, "ros2_control", "mfr3duo.ros2_control.xacro"])
    config = PathJoinSubstitution([share, "config", "controllers.yaml"])
    description = Command(
        [FindExecutable(name="xacro"), " ", xacro,
         " simulation_steps_per_cycle:=", LaunchConfiguration("simulation_steps_per_cycle")]
    )
    return LaunchDescription([
        DeclareLaunchArgument("simulation_steps_per_cycle", default_value="2"),
        DeclareLaunchArgument("controller_update_rate", default_value="500"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[config, {"update_rate": ParameterValue(
                LaunchConfiguration("controller_update_rate"), value_type=int)}],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "imu_broadcaster", "left_arm_controller",
                       "right_arm_controller", "tmr_steering_controller",
                       "tmr_drive_controller", "spine_controller",
                       "--controller-manager-timeout", "120"],
            output="screen",
        ),
    ])
