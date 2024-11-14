import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("microphone_ns")
    namespace_launch_arg = DeclareLaunchArgument("microphone_ns", default_value="audio")

    share_folder = get_package_share_directory("norlab_audio_py")
    config_file = os.path.join(share_folder, "config", "_audio_to_file.yaml")

    microphone_node = Node(
        package="norlab_audio_py",
        name="audio_recorder",
        executable="audio_recorder",
        namespace=namespace,
        output="screen",
        parameters=[config_file],
        remappings=[
            ("audio", "audio"),
        ],
    )

    return LaunchDescription(
        [
            namespace_launch_arg,
            microphone_node,
        ]
    )
