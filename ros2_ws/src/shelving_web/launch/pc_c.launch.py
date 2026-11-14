"""Launch the PC C operator web gateway."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the PC C launch description."""
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "host",
            default_value="0.0.0.0",
            description="Dashboard HTTP bind address.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8080",
            description="Dashboard HTTP port.",
        ),
        Node(
            package="shelving_web",
            executable="web_gateway_node",
            name="web_gateway_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": False,
                    "host": host,
                    "port": ParameterValue(
                        port,
                        value_type=int,
                    ),
                }
            ],
        ),
    ])