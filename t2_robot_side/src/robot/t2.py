from boosteros.robots.booster import BoosterRobot
from .locomotion.control import LocomotionControl
from .manipulation.control import ManipulationControl
from .relay_bridges.imu import ImuRelay
from .relay_bridges.joint_states import JointStatesRelay
from .relay_bridges.odometry import OdometryRelay
from .relay_bridges.tf import TfRelay
from .relay_bridges.tf_static import TfStaticRelay


class T2Robot:
    def __init__(self):
        self.client = BoosterRobot(timeout=30)
        self.locomotion_control = LocomotionControl(self.client)
        self.manipulation_control = ManipulationControl(self.client)

    def launch(self, relay_node):
        self.setup_relay_bridges(relay_node)

    def setup_relay_bridges(self, relay_node):
        self.tf_relay = TfRelay(relay_node)
        self.tf_static_relay = TfStaticRelay(relay_node)
        self.imu_relay = ImuRelay(relay_node)
        self.odometry_relay = OdometryRelay(relay_node)
        self.joint_states_relay = JointStatesRelay(relay_node)

    def locomotion(self, msg, source="schedule"):
        return self.locomotion_control.execute(msg, source)

    def stop_nav2(self):
        self.locomotion_control.stop_nav2()

    def manipulation(self, msg):
        return self.manipulation_control.execute(msg)
