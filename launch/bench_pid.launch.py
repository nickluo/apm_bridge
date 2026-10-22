import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("parameters", default_value=os.path.join(
            get_package_share_directory("apm_bridge"), "parameters", "bench_pid.yaml")),
        DeclareLaunchArgument("enabled", default_value="false", choices=["true", "false"],
                              description="Enable tuning; overrides the YAML enabled value."),
        DeclareLaunchArgument("rig_confirmed", default_value="false", choices=["true", "false"],
                              description="Confirm the captive rig; overrides the YAML rig_confirmed value."),
        Node(package="apm_bridge", executable="bench_pid_tuner", name="bench_pid_tuner",
             output="screen", parameters=[LaunchConfiguration("parameters"), {
                 "enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool),
                 "rig_confirmed": ParameterValue(LaunchConfiguration("rig_confirmed"), value_type=bool),
             }]),
    ])
