from boosteros.robots.booster import BoosterRobot
from ..locomotion.control import LocomotionControl
from ..manipulation.control import ManipulationControl


class T2Robot:
    def __init__(self):
        self.client = BoosterRobot(timeout=30)
        self.locomotion_control = LocomotionControl(self.client)
        self.manipulation_control = ManipulationControl(self.client)

    def launch(self):
        pass

    def locomotion(self, msg):
        return self.locomotion_control.execute(msg)

    def manipulation(self, msg):
        return self.manipulation_control.execute(msg)
