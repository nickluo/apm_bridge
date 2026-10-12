import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_node(context):
    """
    Betaflight custom-link bridge.

    Talks the customized firmware's dedicated high-frequency binary link
    (USE_CUSTOM_LINK, telemetry/custom_link.c) over a serial port (real FC)
    or TCP (SITL simulation), and drives an XFrobot gimbal (default C-20S)
    over the vendor private protocol on a separate UART.

    Betaflight side requirements:
      * firmware built with USE_CUSTOM_LINK (custom_comm branch)
      * the link UART assigned the CUSTOM_LINK function (defaults seed it)
      * OFFBOARD requested through the custom link or a pilot AUX switch;
        firmware safety checks and pilot takeover arbitration remain active
    """
    pkg_share = get_package_share_directory('apm_bridge')
    parameters = [
        os.path.join(pkg_share, 'parameters', 'custom_link_parameters.yaml'),
        {
            'link.transport': LaunchConfiguration('transport'),
            'link.serial_port': LaunchConfiguration('serial_port'),
            'link.baudrate': LaunchConfiguration('baudrate'),
            'link.tcp_host': LaunchConfiguration('tcp_host'),
            'link.tcp_port': LaunchConfiguration('tcp_port'),
        },
        {'gimbal_port': LaunchConfiguration('gimbal_port')},
        {'gimbal.model': LaunchConfiguration('gimbal_model')},
    ]
    pitch_hold = LaunchConfiguration('pitch_hold').perform(context)
    if pitch_hold:
        parameters.append({'gimbal.pitch_hold': ParameterValue(LaunchConfiguration('pitch_hold'), value_type=float)})

    custom_link_node = Node(
        package='apm_bridge',
        executable='custom_link_bridge_node',
        name='custom_link_bridge',
        output='screen',
        parameters=parameters,
        remappings=[
            ('~/low_level_feedback', '/fpv/low_level_feedback'),
            ('~/control_command', '/fpv/control_command'),
            ('~/control_command_raw', '/fpv/control_command_raw'),
            ('~/arm', '/fpv/arm'),
            ('~/offboard', '/fpv/offboard'),
        ]
    )

    return [custom_link_node]


def generate_launch_description():
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
            description='Gimbal serial port. Empty selects a fixed-pitch strapdown camera.'
        ),
        DeclareLaunchArgument(
            'pitch_hold',
            default_value='',
            description='Optional pitch angle (deg): body-relative for strapdown, angle hold for C20S; empty keeps YAML.'
        ),
        DeclareLaunchArgument(
            'gimbal_model',
            default_value='C20S',
            description="Gimbal model: 'C20S' (1-axis), 'C40D' (2-axis) or 'C200T' (3-axis)."
        ),
        OpaqueFunction(function=_launch_node),
    ])
