from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # logger = LaunchConfiguration("log_level")
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     "log_level",
        #     default_value=["info"],
        #     description="Logging level",
        # ),
        Node(
            package="mineros",
            executable="mineros_main",
            name = "mineros_bot",
        )
        # Node(
        #     package="mineros",
        #     executable="info_bot",
        #     name = "info_bot",
        #  )
    ])
