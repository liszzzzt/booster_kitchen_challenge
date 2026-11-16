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
    default_state_file = os.path.join(
        package_share,
        "maps",
        "kitchen",
        "kitchen.pbstream",
    )
    default_map_file = os.path.join(
        package_share,
        "maps",
        "kitchen",
        "kitchen.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    state_file = LaunchConfiguration("state_file")
    map_file = LaunchConfiguration("map_file")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")

    tf_remappings = [
        ("/tf", tf_topic),
        ("/tf_static", tf_static_topic),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("state_file", default_value=default_state_file),
            DeclareLaunchArgument("map_file", default_value=default_map_file),
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
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="t2_cartographer_localization",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    configuration_directory,
                    "-configuration_basename",
                    "cartographer_localization.lua",
                    "-load_state_filename",
                    state_file,
                    "-load_frozen_state=true",
                ],
                remappings=tf_remappings
                + [
                    ("points2", point_cloud_topic),
                    ("imu", imu_topic),
                    ("odom", odom_topic),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"yaml_filename": map_file},
                    {"topic_name": "map"},
                    {"frame_id": "map"},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
        ]
    )
