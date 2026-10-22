import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("parameters", default_value=os.path.join(
            get_package_share_directory("apm_bridge"), "parameters", "bench_pid.yaml")),
        Node(package="apm_bridge", executable="bench_pid_tuner", name="bench_pid_tuner",
             output="screen", parameters=[LaunchConfiguration("parameters")]),
    ])
