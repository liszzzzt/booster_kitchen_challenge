"""Revo2 finger frames in /tf and /tf_static.

    ros2 launch /ws/t2_robot_side/src/robot/manipulation/revo2_tf/revo2_tf.launch.py
    ros2 launch .../revo2_tf.launch.py frame_prefix:=robot1/ robot:=robot1     # multi-robot Studio scene

Starts
  revo2_joint_state.py            /revo2/hand_state -> /revo2/{left,right}/joint_states
  robot_state_publisher  x2       namespaces /revo2/left and /revo2/right, URDF revo2_<side>.urdf
                                  (root <side>_wrist_roll_link, already in Booster's TF tree):
                                  finger joints -> /tf, fixed tip/touch/base links -> /tf_static,
                                  hand URDF on /revo2/<side>/robot_description (not /robot_description)
frame_prefix must match the main tree: "" for a single-robot scene, "robot1/" if Studio prefixes frames.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    frame_prefix = LaunchConfiguration("frame_prefix")
    robot = LaunchConfiguration("robot")
    rate_hz = LaunchConfiguration("rate_hz")
    actions = [
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("robot", default_value="robot1", description='hand_state robot filter ("" = any)'),
        DeclareLaunchArgument("rate_hz", default_value="50.0"),
        ExecuteProcess(
            cmd=["python3", os.path.join(HERE, "revo2_joint_state.py"), "--ros-args",
                 "-p", ["robot:=", robot], "-p", ["rate_hz:=", rate_hz]],
            output="screen", name="revo2_joint_state"),
    ]
    for side in ("left", "right"):
        with open(os.path.join(HERE, "revo2_description", f"revo2_{side}.urdf")) as f:
            urdf = f.read()
        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=f"revo2/{side}", name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": urdf, "frame_prefix": frame_prefix}]))
    return LaunchDescription(actions)
