"""Revo2 finger frames in /t2/tf and /t2/tf_static (Booster Studio and real robot alike).

    ros2 launch /ws/t2_robot_side/src/robot/manipulation/revo2_tf/revo2_tf.launch.py
    ros2 launch .../revo2_tf.launch.py frame_prefix:=robot1/      # only if the TF frames carry a prefix

The T2 agent's JointStatesRelay (relay_bridges/joint_states.py) appends the 22 finger joints, converted
from the hand state, to /t2/joint_states. This starts one robot_state_publisher per hand
(namespace /revo2/<side>, URDF revo2_<side>.urdf with root <side>_wrist_roll_link, which Booster's
TF already has); each reads /t2/joint_states (it uses only its hand's joints) and publishes
    finger links (moving)                                          -> /t2/tf
    palm, fingertip / pad points, power / pinch grasp frames       -> /t2/tf_static
The hand URDF goes to /revo2/<side>/robot_description (not /robot_description).
Requires the T2 agent to be running; no finger TF without it.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def _nodes(context):
    prefix = LaunchConfiguration("frame_prefix").perform(context)
    nodes = []
    for side in ("left", "right"):
        with open(os.path.join(HERE, "revo2_description", f"revo2_{side}.urdf")) as f:
            urdf = f.read()
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=f"revo2/{side}", name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": urdf, "frame_prefix": prefix}],
            remappings=[("joint_states", "/t2/joint_states"), ("/tf", "/t2/tf"), ("/tf_static", "/t2/tf_static")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frame_prefix", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
