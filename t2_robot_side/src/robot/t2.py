import threading

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
        self.motion_state_lock = threading.Lock()
        self.motion_state = None

    def launch(self, relay_node):
        self.setup_relay_bridges(relay_node)

    def setup_relay_bridges(self, relay_node):
        self.tf_relay = TfRelay(relay_node)
        self.tf_static_relay = TfStaticRelay(relay_node)
        self.imu_relay = ImuRelay(relay_node)
        self.odometry_relay = OdometryRelay(relay_node)
        self.joint_states_relay = JointStatesRelay(relay_node)

    def locomotion(self, msg, source="schedule"):
        with self.motion_state_lock:
            # manipulation is running, abort locomotion
            if self.motion_state == "manipulation":
                return False
            self.motion_state = "locomotion"

        try:
            return self.locomotion_control.execute(msg, source)
        finally:
            with self.motion_state_lock:
                if self.motion_state == "locomotion":
                    self.motion_state = None

    def stop_nav2(self):
        self.locomotion_control.stop_nav2()

    def manipulation(self, msg):
        with self.motion_state_lock:
            # manipulation or locomotion is running, abort manipulation
            if self.motion_state is not None:
                return False
            self.motion_state = "manipulation"

        try:
            return self.manipulation_control.execute(msg)
        finally:
            with self.motion_state_lock:
                self.motion_state = None

    def stop_manipulation(self):
        self.manipulation_control.stop()
