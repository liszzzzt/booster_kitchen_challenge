import time
from t2_server_side.high_level_planner.schedule.kitchen_challenge import schedule


class CenterControl():
    def __init__(self, server_node):
        self.server_node = server_node

    def execute_schedule(self):
        for command_id, command in schedule.items():
            print(command_id, command)
            if 'walk' in command:
                self.server_node.send_locomotion_command({
                    "id": command_id,
                    "target": "simpleTable",
                    "frame_id": "map",
                    "coordinate": [2, 1, 0, 0, 0, 1.57]
                })
            elif 'pick' in command:
                self.server_node.send_manipulation_command({
                    "id": command_id,
                    "target": "cup",
                    "frame_id": "camera",
                    "coordinate": [0.3, -0.02, 0.1, 0.8, 1, 1.57]
                })

            success = self.server_node.wait_for_success(command_id)
            if not success:
                print(f"Command {command_id} failed. Schedule stopped.")
                break

            print(f"Command {command_id} completed.")
