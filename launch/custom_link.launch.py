import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Betaflight custom-link bridge.

    Same ROS interface as the APM bridge, but talks the firmware's dedicated
    high-frequency binary link (USE_CUSTOM_LINK, telemetry/custom_link.c) over
    a serial port (real FC) or TCP (SITL simulation) instead of MAVLink.

    Betaflight side requirements:
      * firmware built with USE_CUSTOM_LINK (custom_comm branch)
      * the link UART assigned the CUSTOM_LINK function (defaults seed it)
      * an AUX switch assigned to OFFBOARD - the host can only arm/take over
        while the pilot holds that switch
    """
    pkg_share = get_package_share_directory('apm_bridge')

    custom_link_node = Node(
        package='apm_bridge',
        executable='custom_link_bridge_node',
        name='custom_link_bridge',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'parameters', 'custom_link_parameters.yaml'),
            {
                'link.transport': LaunchConfiguration('transport'),
                'link.serial_port': LaunchConfiguration('serial_port'),
                'link.baudrate': LaunchConfiguration('baudrate'),
                'link.tcp_host': LaunchConfiguration('tcp_host'),
                'link.tcp_port': LaunchConfiguration('tcp_port'),
            },
            {'gimbal_port': LaunchConfiguration('gimbal_port')},
        ],
        remappings=[
            ('~/low_level_feedback', '/fpv/low_level_feedback'),
            ('~/control_command', '/fpv/control_command'),
            ('~/control_command_raw', '/fpv/control_command_raw'),
            ('~/arm', '/fpv/arm'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'transport',
            default_value='serial',
            description="'serial' for a real FC UART, 'tcp' for Betaflight SITL."
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyTHS1',
            description='Serial port wired to the FC link UART (FUNCTION_CUSTOM_LINK).'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='921600',
            description='Link baudrate; must match the FC port configuration.'
        ),
        DeclareLaunchArgument(
            'tcp_host', default_value='127.0.0.1', description='SITL host.'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='5763',
            description='SITL TCP port of the link UART (UART3 = 5763).'
        ),
        DeclareLaunchArgument(
            'gimbal_port',
            default_value='',
            description='Gimbal serial port. Empty disables the gimbal thread.'
        ),
        custom_link_node,
    ])
