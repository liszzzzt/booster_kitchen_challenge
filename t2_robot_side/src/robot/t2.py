from boosteros.robots.booster import BoosterRobot


class T2Robot:
    def __init__(self):
        self.client = BoosterRobot(timeout=30)