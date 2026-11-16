from t2_server_side.high_level_planner.schedule.kitchen_challenge import (
    navigation_goals,
    schedule,
)


class CenterControl():
    def __init__(self, server_node, nav2_client):
        self.server_node = server_node
        self.nav2_client = nav2_client

    def execute_schedule(self):
        for command_id, command in schedule.items():
            print(command_id, command)
            success = False
            if 'walk' in command:
                target = command.split()[-1]
                x, y, yaw = navigation_goals[target]
                print(f"Nav2: [{x}, {y}, {yaw}]")
                success = self.nav2_client.navigate_to(x, y, yaw)
                # self.server_node.send_locomotion_command({
                #     "id": command_id,
                #     "target": "simpleTable",
                #     "frame_id": "map",
                #     "coordinate": [2, 1, 0, 0, 0, 1.57]
                # })
            elif 'pick' in command:
                print(f"Manipulation: [{command}]")
                self.server_node.send_manipulation_command({
                    "id": command_id,
                    "action": "pick",
                    "target": "cup",
                    "frame_id": "camera",
                    "coordinate": [0.3, -0.02, 0.1, 0.8, 1, 1.57]
                })
                success = self.server_node.wait_for_success(command_id)

            if not success:
                print(f"Command {command_id} failed. Schedule stopped.")
                break

            print(f"Command {command_id} completed.")
