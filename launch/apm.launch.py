import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the APM MAVROS node.

    This function is the entry point for the ROS2 launch system. It sets up
    all the necessary launch arguments and defines the MAVROS node to be
    executed. This Python launch file is a conversion of the original
    apm.launch and node.launch XML files.
    """
    # Find the package share directory for apm_bridge to locate config files
    apm_bridge_pkg_share = get_package_share_directory('apm_bridge')

    # Define the paths to the YAML configuration files
    pluginlists_yaml_path = os.path.join(apm_bridge_pkg_share, 'parameters', 'apm_pluginlists.yaml')
    config_yaml_path = os.path.join(apm_bridge_pkg_share, 'parameters', 'apm_config.yaml')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        # In ROS2 Python launch files, parameters are passed as a list.
        # This includes both direct key-value parameters and parameter files.
        parameters=[
            # Parameters loaded from YAML files, equivalent to <param from="...">
            pluginlists_yaml_path,
            config_yaml_path,

            # Direct parameters, equivalent to <param name="..." value="...">
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'gcs_url': LaunchConfiguration('gcs_url')},
            {'tgt_system': LaunchConfiguration('tgt_system')},
            {'tgt_component': LaunchConfiguration('tgt_component')},
            {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
        ],
        remappings=[
            ('/mavros/local_position/odom', '/fpv/odom'),
        ],
        # The respawn functionality is handled by the launch system itself
        respawn=LaunchConfiguration('respawn_mavros'),
    )

    # APM Bridge Node
    apm_bridge_node = Node(
        package='apm_bridge',
        executable='apm_bridge_node',
        name='apm_bridge',
        output='screen',
        parameters=[
            os.path.join(apm_bridge_pkg_share, 'parameters', 'airsim_parameters.yaml'),
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
        # --- Declare Launch Arguments ---
        # These correspond to the <arg> tags in the original XML launch files.
        DeclareLaunchArgument(
            'gimbal_port',
            default_value='/dev/ttyTHS2',
            description='Gimbal connection port.'
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyTHS1:921600',
            description='FCU connection URL.'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS connection URL.'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target MAVLink system ID.'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target MAVLink component ID.'
        ),
        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='Type of logging output.'
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVLink protocol version.'
        ),
        DeclareLaunchArgument(
            'respawn_mavros',
            default_value='false',
            description='Whether to respawn mavros node on exit.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='mavros',
            description='Namespace for the mavros node.'
        ),

        # --- Define the Node ---
        # This corresponds to the <node> tag in node.launch.
        # The logic from both apm.launch and node.launch is combined here.
        mavros_node,
        apm_bridge_node
        
    ])
