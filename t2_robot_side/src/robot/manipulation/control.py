import json

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
            pass
            # return self.finger_control.execute(msg)

        return False

    def stop(self):
        self.moveit2_control.stop()
