import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_directory = os.path.join(
        get_package_share_directory("t2_ros2_bringup"),
        "launch",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")

    common_arguments = {
        "use_sim_time": use_sim_time,
        "tf_topic": tf_topic,
        "tf_static_topic": tf_static_topic,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("tf_topic", default_value="/t2/tf"),
            DeclareLaunchArgument(
                "tf_static_topic",
                default_value="/t2/tf_static",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_directory, "cartographer.launch.py")
                ),
                launch_arguments=common_arguments.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_directory, "rviz2.launch.py")
                ),
                launch_arguments=common_arguments.items(),
            ),
        ]
    )
