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
        #Node(
        #    package="my_first_node",
        #    executable="fsm",
        #    name = "fsm_node",
        # ),
        #Node(
        #    package="my_first_node",
        #    executable="helper",
        #    name = "helper_node",
        #),
        Node(
            package="mineros_testing",
            executable="movement_tester",
            name = "movement_tester",
        ),
    ])
