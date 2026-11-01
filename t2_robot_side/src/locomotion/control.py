import time


class LocomotionController:
    def __init__(self, robot):
        self.robot = robot

    def test_walk(self):
        self.robot.client.set_mode("walk")

        try:
            self.robot.client.set_velocity(0.2, 0.0, 0.0)
            time.sleep(3.0)
        finally:
            self.robot.client.set_velocity(0.0, 0.0, 0.0)