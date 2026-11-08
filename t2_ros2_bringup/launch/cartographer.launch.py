import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("t2_ros2_bringup")
    configuration_directory = os.path.join(
        package_share,
        "config",
        "cartographer",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    configuration_basename = LaunchConfiguration("configuration_basename")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    tf_remapping = [
        ("/tf", tf_topic),
        ("/tf_static", tf_static_topic),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="cartographer_mapping.lua",
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="/e1r/points",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/t2/imu/data",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/t2/odom",
            ),
            DeclareLaunchArgument("tf_topic", default_value="/t2/tf"),
            DeclareLaunchArgument(
                "tf_static_topic",
                default_value="/t2/tf_static",
            ),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="t2_cartographer",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    configuration_directory,
                    "-configuration_basename",
                    configuration_basename,
                ],
                remappings=tf_remapping
                + [
                    ("points2", point_cloud_topic),
                    ("imu", imu_topic),
                    ("odom", odom_topic),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="t2_occupancy_grid",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-resolution",
                    resolution,
                    "-publish_period_sec",
                    publish_period_sec,
                ],
                remappings=tf_remapping,
            ),
        ]
    )
