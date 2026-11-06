from boosteros.robots.booster import BoosterRobot
from ..locomotion.control import LocomotionControl
from ..manipulation.control import ManipulationControl
from .relay_bridges.imu import ImuRelay
from .relay_bridges.joint_states import JointStatesRelay
from .relay_bridges.odometry import OdometryRelay
from .relay_bridges.tf import TfRelay


class T2Robot:
    def __init__(self):
        self.client = BoosterRobot(timeout=30)
        self.locomotion_control = LocomotionControl(self.client)
        self.manipulation_control = ManipulationControl(self.client)

    def launch(self, node):
        self.setup_relay_bridges(node)

    def setup_relay_bridges(self, node):
        self.tf_relay = TfRelay(node)
        self.imu_relay = ImuRelay(node)
        self.odometry_relay = OdometryRelay(node)
        self.joint_states_relay = JointStatesRelay(node)

    def locomotion(self, msg):
        return self.locomotion_control.execute(msg)

    def manipulation(self, msg):
        return self.manipulation_control.execute(msg)
