import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Values are in radians and ordered joint1 through joint7. The corresponding
# angles in degrees are [0, 20, 0, 70, 0, 90, 90].
INITIAL_JOINT_POSITIONS = [
    0.0,
    0.3490658503988659,
    0.0,
    1.2217304763960306,
    0.0,
    1.5707963267948966,
    1.5707963267948966,
]


def generate_launch_description():
    move_to_initial_pose = LaunchConfiguration("move_to_initial_pose")

    rm_75_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_driver"),
                "launch",
                "rm_75_driver.launch.py",
            )
        )
    )

    rm_75_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_description"),
                "launch",
                "rm_75_display.launch.py",
            )
        ),
    )

    rm_75_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_control"),
                "launch",
                "rm_75_control.launch.py",
            )
        )
    )

    rm_75_moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_75_config"),
                "launch",
                "real_moveit_demo.launch.py",
            )
        )
    )

    initial_pose_node = Node(
        package="rm_bringup",
        executable="rm_75_initial_pose",
        name="rm_75_initial_pose",
        parameters=[{"initial_joint_positions": INITIAL_JOINT_POSITIONS}],
        output="screen",
        condition=IfCondition(move_to_initial_pose),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "move_to_initial_pose",
                default_value="false",
                description=(
                    "Whether to move RM75 to the configured initial joint pose "
                    "at startup."
                ),
            ),
            rm_75_driver,
            rm_75_description,
            rm_75_control,
            rm_75_moveit_config,
            initial_pose_node,
        ]
    )
