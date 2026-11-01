import threading
from booster_agent_framework import AgentBase, AgentFeatures
from locomotion.control import LocomotionController
from robot.t2 import T2Robot


class T2KitchenChallengeAgent(AgentBase):
    def __init__(self):
        super().__init__(AgentFeatures())

        self.t2 = T2Robot()
        self.locomotion = LocomotionController(self.t2)

        threading.Thread(
            target=self.locomotion.test_walk,
            daemon=True,
        ).start()