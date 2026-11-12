import threading
import time

from boosteros.types import JointCommand
from control_msgs.action import FollowJointTrajectory


class MoveIt2Control:
    CONTROL_PERIOD = 0.02

    HEAD_JOINTS = [
        "aa_head_yaw_joint",
        "head_pitch_joint",
    ]
    ARM_JOINTS = [
        "shoulder_pitch",
        "shoulder_roll",
        "elbow_pitch",
        "elbow_yaw",
        "wrist_pitch",
        "wrist_yaw",
        "wrist_roll",
    ]
    LEFT_ARM_JOINTS = [f"left_{name}_joint" for name in ARM_JOINTS]
    RIGHT_ARM_JOINTS = [f"right_{name}_joint" for name in ARM_JOINTS]
    UPPER_BODY_JOINTS = HEAD_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

    def __init__(self, client):
        self.client = client
        self.stop_requested = threading.Event()
        self.enabled = False
        self.upper_body_positions = {}

    def execute(self, command):
        goal_handle = command["goal_handle"]
        # left | right
        arm_side = command["arm_side"]
        # seven joint names
        joint_names = self.get_joint_names(arm_side)
        # [
        #   (time=t0, positions=[j1, j2, j3, j4, j5, j6, j7]),
        #   (time=t1, positions=[j1, j2, j3, j4, j5, j6, j7]),
        #   ...
        # ]
        # time: time_from_start
        trajectory = goal_handle.request.trajectory

        self.stop_requested.clear()
        self.start()
        completed = self.run(goal_handle, joint_names, trajectory)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        if completed:
            goal_handle.succeed()
        else:
            goal_handle.canceled()
            result.error_string = "Trajectory canceled"
        return result

    def get_joint_names(self, arm_side):
        if arm_side == "left":
            return self.LEFT_ARM_JOINTS
        return self.RIGHT_ARM_JOINTS

    def start(self):
        if self.client.get_mode() != "walk":
            self.client.set_mode("walk")
            time.sleep(1.0)

        if not self.upper_body_positions:
            self.upper_body_positions = {
                name: position
                for name, position in self.get_joint_positions().items()
                if name in self.UPPER_BODY_JOINTS
            }

        if not self.enabled:
            self.client.upper_body_control(True)
            self.enabled = True
            self.send_targets()

    def run(self, goal_handle, joint_names, trajectory):
        indices = [trajectory.joint_names.index(name) for name in joint_names]
        previous_positions = [self.upper_body_positions[name] for name in joint_names]
        previous_time = 0.0
        start_time = time.monotonic()

        for point in trajectory.points:
            point_time = self.duration_seconds(point.time_from_start)
            moveit_waypoint_positions = [point.positions[index] for index in indices]
            segment_duration = point_time - previous_time

            while time.monotonic() - start_time < point_time:
                if goal_handle.is_cancel_requested or self.stop_requested.is_set():
                    return False

                elapsed = time.monotonic() - start_time
                alpha = (elapsed - previous_time) / segment_duration
                alpha = max(0.0, min(1.0, alpha))
                next_interpolated_positions = [
                    start + alpha * (goal - start) for start, goal in zip(previous_positions, moveit_waypoint_positions)
                ]
                self.command_arm(joint_names, next_interpolated_positions)
                time.sleep(self.CONTROL_PERIOD)

            self.command_arm(joint_names, moveit_waypoint_positions)
            previous_positions = moveit_waypoint_positions
            previous_time = point_time

        return True

    def command_arm(self, joint_names, positions):
        self.upper_body_positions.update(zip(joint_names, positions))
        self.send_targets()

    def send_targets(self):
        GAINS = {
            "aa_head_yaw_joint": (40.0, 2.0),
            "head_pitch_joint": (40.0, 2.0),
            "left_shoulder_pitch_joint": (20.0, 1.5),
            "left_shoulder_roll_joint": (30.0, 2.0),
            "left_elbow_pitch_joint": (10.0, 0.5),
            "left_elbow_yaw_joint": (10.0, 0.5),
            "left_wrist_pitch_joint": (20.0, 1.5),
            "left_wrist_yaw_joint": (30.0, 2.0),
            "left_wrist_roll_joint": (10.0, 0.5),
            "right_shoulder_pitch_joint": (20.0, 1.5),
            "right_shoulder_roll_joint": (30.0, 2.0),
            "right_elbow_pitch_joint": (10.0, 0.5),
            "right_elbow_yaw_joint": (10.0, 0.5),
            "right_wrist_pitch_joint": (20.0, 1.5),
            "right_wrist_yaw_joint": (30.0, 2.0),
            "right_wrist_roll_joint": (10.0, 0.5),
        }

        commands = []
        for name in self.UPPER_BODY_JOINTS:
            kp, kd = GAINS[name]
            commands.append(
                JointCommand(
                    name=name,
                    position=float(self.upper_body_positions[name]),
                    kp=kp,
                    kd=kd,
                )
            )
        self.client.set_joints(commands)

    def get_joint_positions(self):
        return {
            state.name: float(state.position)
            for state in self.client.get_joint_states().joints
        }

    def stop(self):
        self.stop_requested.set()
        if self.enabled:
            self.client.upper_body_control(False)
            self.enabled = False

    @staticmethod
    def duration_seconds(duration):
        return duration.sec + duration.nanosec * 1e-9
