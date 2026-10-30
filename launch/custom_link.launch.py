import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Node parameters default from parameters/custom_link_parameters.yaml; the
# launch arguments below override individual values only when explicitly
# given (their empty default means "keep the YAML value").
# (launch argument, node parameter, parameter type)
_ARG_OVERRIDES = (
    ('transport', 'link.transport', str),
    ('serial_port', 'link.serial_port', str),
    ('baudrate', 'link.baudrate', int),
    ('tcp_host', 'link.tcp_host', str),
    ('tcp_port', 'link.tcp_port', int),
    ('gimbal_port', 'gimbal_port', str),
    ('gimbal_model', 'gimbal.model', str),
    ('pitch_hold', 'gimbal.pitch_hold', float),
    ('pitch_aim', 'gimbal.pitch_aim', float),
    ('bind_rx', 'cli.bind_rx_on_connect', bool),
)


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

    Parameter defaults live in parameters/custom_link_parameters.yaml; the
    declared launch arguments override individual values only when
    explicitly given on the command line (empty keeps the YAML value).
    """
    pkg_share = get_package_share_directory('apm_bridge')
    # mixed parameter sources: the YAML file plus optional override dicts
    parameters: list = [
        os.path.join(pkg_share, 'parameters', 'custom_link_parameters.yaml'),
    ]
    for arg, param, value_type in _ARG_OVERRIDES:
        if LaunchConfiguration(arg).perform(context):
            parameters.append({param: ParameterValue(LaunchConfiguration(arg), value_type=value_type)})

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
            default_value='',
            description="'serial' for a real FC UART, 'tcp' for Betaflight SITL; "
                        'empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Serial port wired to the FC link UART (FUNCTION_CUSTOM_LINK); '
                        'empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='',
            description='Link baudrate; must match the FC port configuration; '
                        'empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'tcp_host', default_value='',
            description='SITL host; empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'tcp_port', default_value='',
            description='SITL TCP port of the link UART (UART3 = 5763); '
                        'empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'gimbal_port',
            default_value='',
            description='Gimbal serial port (empty YAML value selects a fixed-pitch '
                        'strapdown camera); empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'pitch_hold',
            default_value='',
            description='Optional pitch angle (deg): body-relative for strapdown, angle hold for C20S; empty keeps YAML.'
        ),
        DeclareLaunchArgument(
            'gimbal_model',
            default_value='',
            description="Gimbal model: 'C20S' (1-axis), 'C40D' (2-axis) or 'C200T' (3-axis); "
                        'empty keeps the custom_link_parameters.yaml value.'
        ),
        DeclareLaunchArgument(
            'pitch_aim', default_value='',
            description='Optional AIM offset (deg, nose-up): body-relative target is 90 + pitch_aim, subject to limits; empty keeps YAML.'
        ),
        DeclareLaunchArgument(
            'bind_rx', default_value='', choices=['', 'true', 'false'],
            description='Override cli.bind_rx_on_connect: enter RX pairing mode on every '
                        'link up. Keep false except during an actual pairing session '
                        '(the RX stays in bind until a full power cycle); empty keeps YAML.'
        ),
        OpaqueFunction(function=_launch_node),
    ])
