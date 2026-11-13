"""Revo2 finger frames in /tf and /tf_static.

    ros2 launch /ws/t2_robot_side/src/robot/manipulation/revo2_tf/revo2_tf.launch.py               # Studio
    ros2 launch .../revo2_tf.launch.py topics:=t2                                                   # real robot
    ros2 launch .../revo2_tf.launch.py frame_prefix:=robot1/ robot:=robot1     # multi-robot Studio scene

topics:=sim (default, Booster Studio; T2WS shares the sim's DDS):
    in /revo2/hand_state + /joint_states, out /tf + /tf_static (the robot-side TfRelay then copies the
    finger frames to /t2/tf like every other frame)
topics:=t2 (real robot; T2WS only sees the robot-side relays):
    in /t2/revo2/hand_state (Revo2Relay) + /t2/joint_states, out /t2/tf + /t2/tf_static
Use one mode only; running both against Studio publishes every finger frame twice on /t2/tf.

Starts
  revo2_joint_state.py            hand_state -> /revo2/{left,right}/joint_states
  robot_state_publisher  x2       namespaces /revo2/left and /revo2/right, URDF revo2_<side>.urdf
                                  (root <side>_wrist_roll_link, already in Booster's TF tree):
                                  finger joints -> tf, fixed tip/touch/base links -> tf_static,
                                  hand URDF on /revo2/<side>/robot_description (not /robot_description)
frame_prefix must match the main tree: "" for a single-robot scene, "robot1/" if Studio prefixes frames.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
TOPICS = {
    "sim": dict(hand_state="/revo2/hand_state", sync="/joint_states", tf="/tf", tf_static="/tf_static"),
    "t2": dict(hand_state="/t2/revo2/hand_state", sync="/t2/joint_states", tf="/t2/tf", tf_static="/t2/tf_static"),
}


def _nodes(context):
    mode = LaunchConfiguration("topics").perform(context)
    if mode not in TOPICS:
        raise ValueError(f"topics must be one of {sorted(TOPICS)}, got {mode!r}")
    t = TOPICS[mode]
    actions = [ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "revo2_joint_state.py"), "--ros-args",
             "-p", f"robot:={LaunchConfiguration('robot').perform(context)}",
             "-p", f"rate_hz:={LaunchConfiguration('rate_hz').perform(context)}",
             "-p", f"hand_state_topic:={t['hand_state']}",
             "-p", f"sync_topic:={t['sync']}"],
        output="screen", name="revo2_joint_state")]
    for side in ("left", "right"):
        with open(os.path.join(HERE, "revo2_description", f"revo2_{side}.urdf")) as f:
            urdf = f.read()
        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=f"revo2/{side}", name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": urdf,
                         "frame_prefix": LaunchConfiguration("frame_prefix").perform(context)}],
            remappings=[("/tf", t["tf"]), ("/tf_static", t["tf_static"])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topics", default_value="sim", description="sim (Booster Studio) | t2 (real robot)"),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("robot", default_value="robot1", description='hand_state robot filter ("" = any)'),
        DeclareLaunchArgument("rate_hz", default_value="50.0"),
        OpaqueFunction(function=_nodes),
    ])
