import time
import json
import threading

from .nav2_control import Nav2Control
from .remote_control import LocomotionRemoteControl


class LocomotionControl():
    def __init__(self, client):
        self.client = client
        self.remote_control = LocomotionRemoteControl(self.client)
        self.nav2_control = Nav2Control(self.client)
        self.lock = threading.Lock()
        self.active_source = None

    def execute(self, msg, source="schedule"):
        if source == "nav2":
            with self.lock:
                if self.active_source == "remote":
                    return False

                if self.active_source != "nav2":
                    self.nav2_control.start()
                self.active_source = "nav2"
                return self.nav2_control.execute(msg)

        # {"id": 1, "target": "simpleTable", "frame_id": "map", "coordinate": [2, 1, 0, 0, 0, 1.57]}
        # {"id": 1, "target": "", "frame_id": "map", "coordinate": [], "type": "remote", "key": "w"}
        command = json.loads(msg.data)
        command_type = command.get("type", "")

        if command_type == "remote":
            key = command["key"]

            if key == "=":
                # resume nav2
                with self.lock:
                    self.nav2_control.start()
                    self.active_source = "nav2"
                return True

            with self.lock:
                self.active_source = "remote"
            return self.remote_control.execute(key)

        with self.lock:
            # one time, one schedule
            if self.active_source in ("remote", "nav2"):
                return False
            self.active_source = "schedule"
        try:
            return self.execute_schedule()
        finally:
            with self.lock:
                self.active_source = None

    def stop_nav2(self):
        with self.lock:
            if self.active_source == "nav2":
                self.nav2_control.stop()

    def execute_schedule(self):
        self.client.set_mode("walk")

        try:
            self.client.set_velocity(0.2, 0.0, 0.0)
            time.sleep(8.0)
        finally:
            self.client.set_velocity(0.0, 0.0, 0.0)

        return True
