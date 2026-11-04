import sys
import termios
import tty
import time
import rclpy

def read_key(prompt=""):
    print(prompt, end="", flush=True)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings,
        )

    print(key)
    return key.lower()


class RemoteControl:
    def __init__(self, server_node):
        self.server_node = server_node

    def launch(self):
        LOCOMOTION_KEYS = {
            "w", "s", "a", "d", "q", "e",
            "z", "x", "c", "v", "b", "n",
            "r", "t", "y", "u",
        }
        MANIPULATION_KEYS = {
            "1", "2", "3", "4", "5", "6",
            "7", "8", "9", "0", "-", "=",
        }

        command_id = 1
        while True:
            key = read_key()
            msg = {
                "id": command_id,
                "target": "",
                "frame_id": "map",
                "coordinate": [],
                "type": "remote",
                "key": key,
                "action": "",
                "level": "",
            }
            if key in LOCOMOTION_KEYS:
                self.server_node.send_locomotion_command(msg)
            elif key in MANIPULATION_KEYS:
                self.server_node.send_manipulation_command(msg)
            elif key == 'p':
                # test
                msg = {
                    "id": command_id,
                    "level": 1,
                    "type": "posture",
                    "action": "crouch",
                }
                self.server_node.send_locomotion_command(msg)
            else:
                print(f"Unknown key: {key}")
                continue
            success = self.server_node.wait_for_success(command_id)
            if success:
                print(f"Command {command_id} completed. Key: {key}")
            else:
                print(f"Command {command_id} failed. Key: {key}")
            command_id = command_id + 1
