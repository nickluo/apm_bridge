import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("apm_bridge")
    return LaunchDescription([
        DeclareLaunchArgument("parameters", default_value=os.path.join(
            share, "parameters", "sitl_parameters.yaml")),
        DeclareLaunchArgument("airsim_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("airsim_port", default_value="41451"),
        DeclareLaunchArgument("vehicle_name", default_value="Copter"),
        DeclareLaunchArgument("tcp_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tcp_port", default_value="5763"),
        DeclareLaunchArgument("airsim_link_port", default_value="5764"),
        DeclareLaunchArgument("camera_name", default_value="0"),
        DeclareLaunchArgument("gimbal_enabled", default_value="true"),
        Node(
            package="apm_bridge", executable="custom_link_bridge_node", name="custom_link_bridge",
            output="screen",
            parameters=[LaunchConfiguration("parameters"), {
                "airsim.host": ParameterValue(LaunchConfiguration("airsim_host"), value_type=str),
                "airsim.port": ParameterValue(LaunchConfiguration("airsim_port"), value_type=int),
                "airsim.vehicle_name": ParameterValue(
                    LaunchConfiguration("vehicle_name"), value_type=str),
                "link.tcp_host": ParameterValue(LaunchConfiguration("tcp_host"), value_type=str),
                "link.tcp_port": ParameterValue(LaunchConfiguration("tcp_port"), value_type=int),
                "airsim.link_port": ParameterValue(LaunchConfiguration("airsim_link_port"), value_type=int),
                "airsim.camera_name": ParameterValue(
                    LaunchConfiguration("camera_name"), value_type=str),
                "airsim.gimbal_enabled": ParameterValue(
                    LaunchConfiguration("gimbal_enabled"), value_type=bool),
            }],
            remappings=[
                ("~/low_level_feedback", "/fpv/low_level_feedback"),
                ("~/control_command", "/fpv/control_command"),
                ("~/control_command_raw", "/fpv/control_command_raw"),
                ("~/arm", "/fpv/arm"),
                ("~/offboard", "/fpv/offboard"),
            ],
        ),
    ])
