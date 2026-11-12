import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("t2_description")
    moveit_share = get_package_share_directory("t2_moveit_config")

    description_file = os.path.join(
        description_share,
        "urdf",
        "t2.urdf.xacro",
    )
    rviz_config = os.path.join(moveit_share, "rviz", "moveit.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("rviz")

    moveit_config = (
        MoveItConfigsBuilder("t2", package_name="t2_moveit_config")
        .robot_description(file_path=description_file)
        .robot_description_semantic(file_path="config/t2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=False,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    tf_remappings = [
        ("/tf", "/t2/tf"),
        ("/tf_static", "/t2/tf_static"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "use_sim_time": use_sim_time,
                        "moveit_manage_controllers": False,
                    },
                ],
                remappings=[
                    ("/joint_states", "/t2/joint_states"),
                    *tf_remappings,
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="t2_moveit_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=tf_remappings,
            ),
        ]
    )
