import time
import json
from .remote_control import LocomotionRemoteControl


class LocomotionControl():
    def __init__(self, client):
        self.client = client
        self.remote_control = LocomotionRemoteControl()

    def execute(self, msg):
        # {"id": 1, "target": "simpleTable", "frame_id": "map", "coordinate": [2, 1, 0, 0, 0, 1.57]}
        # {"id": 1, "target": "", "frame_id": "map", "coordinate": [], "type": "remote", "key": "w"}
        command = json.loads(msg.data)
        command_type = command.get("type", "")

        if command_type == "remote":
            return True
        if command_type == "posture":
            return True

        return self.walk_forward()

    def walk_forward(self):
        self.client.set_mode("walk")

        try:
            self.client.set_velocity(0.2, 0.0, 0.0)
            time.sleep(3.0)
        finally:
            self.client.set_velocity(0.0, 0.0, 0.0)

        return True
