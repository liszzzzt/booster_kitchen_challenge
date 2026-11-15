import json
import time

from boosteros.types import JointCommand
from .moveit2_control import MoveIt2Control


class ManipulationControl():
    def __init__(self, client):
        self.client = client
        self.moveit2_control = MoveIt2Control(client)

    def execute(self, msg):
        command = msg if isinstance(msg, dict) else json.loads(msg.data)
        command_type = command.get("type", "")

        if command_type == "moveit2":
            return self.moveit2_control.execute(command)

        if command_type == "finger":
            return False
            # return self.finger_control.execute(msg)

        return self.execute_schedule()

    def stop(self):
        self.moveit2_control.stop()

    def execute_schedule(self):
        # boosteros_examples: joint_upper_control_example.py
        self.moveit2_control.stop()

        gains = {
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

        self.client.set_mode("walk")
        time.sleep(1.0)

        measured = {
            state.name: float(state.position)
            for state in self.client.get_joint_states().joints
        }

        commands = {
            name: JointCommand(
                name=name,
                position=measured[name],
                kp=kp,
                kd=kd,
            )
            for name, (kp, kd) in gains.items()
        }

        joint_name = "left_shoulder_pitch_joint"
        start_position = commands[joint_name].position

        upper_body_enabled = False
        try:
            self.client.upper_body_control(True)
            upper_body_enabled = True

            # Hold the measured starting posture.
            self.client.set_joints(list(commands.values()))
            time.sleep(1.0)

            # 0.2 rad = 11.5 degrees, 0.4 rad = 22.9 degrees
            commands[joint_name].position = start_position + 0.40
            print(
                f"official upper-body test: {joint_name} "
                f"{start_position:.3f} -> {commands[joint_name].position:.3f}",
                flush=True,
            )
            self.client.set_joints(list(commands.values()))
            time.sleep(2.0)

            # Return to the original posture.
            commands[joint_name].position = start_position
            self.client.set_joints(list(commands.values()))
            time.sleep(2.0)

            return True
        finally:
            if upper_body_enabled:
                self.client.upper_body_control(False)
