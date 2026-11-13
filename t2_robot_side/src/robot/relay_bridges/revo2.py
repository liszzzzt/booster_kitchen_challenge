"""Real-robot Revo2 hands <-> server, std_msgs/String JSON (same format as the Studio revo2_hand plugin).

  in   /t2/revo2/hand_cmd    {"side": "left"|"right", "closure": {dof: 0..1}}   0 = open, 1 = closed;
                             dofs: thumb_rotation thumb_flex index middle ring pinky; unlisted dofs keep
                             their last target
  out  /t2/revo2/hand_state  {"side", "closure": {dof: 0..1}, "time"}  20 Hz per hand

The hand is not a ROS topic on the robot: commands are the SDK RPC ControlDexterousHand and the state is
rt/booster_hand_data, whose message type ROS on the server does not have. So this converts instead of
relaying. Only active when GetHands() reports hands (real robot); idle in Booster Studio.

SEQ / ANGLE_* follow Booster's Inspire RH56 convention; verify them on the real Revo2 before grasping.
"""
import json
import queue
import threading

from std_msgs.msg import String

DOFS = ("thumb_rotation", "thumb_flex", "index", "middle", "ring", "pinky")
SEQ = dict(pinky=0, ring=1, middle=2, index=3, thumb_flex=4, thumb_rotation=5)   # VERIFY on the Revo2
ANGLE_OPEN = 1000                                                                 # VERIFY
ANGLE_CLOSED = 0                                                                  # VERIFY
FORCE = 500
SPEED = 500


class Revo2Relay:
    def __init__(self, node, client):
        self.node = node
        if not self._has_hands(client):
            node.get_logger().info("revo2 relay: no hands reported, off")
            return
        import booster_robotics_sdk_python as b

        self.b = b
        self.loco = client.loco_client
        self.hand_index = {"left": b.HandIndex.kLeftHand, "right": b.HandIndex.kRightHand}
        self.lock = threading.Lock()
        self.targets = {side: {dof: 0.0 for dof in DOFS} for side in self.hand_index}
        self.closure = {}

        self.publisher = node.create_publisher(String, "/t2/revo2/hand_state", 10)
        self.subscription = node.create_subscription(String, "/t2/revo2/hand_cmd", self._on_command, 10)
        self.timer = node.create_timer(0.05, self._publish_state)

        self.state_subscriber = b.B1LowHandDataSubscriber(self._on_hand_data)
        self.state_subscriber.InitChannel()

        # ControlDexterousHand blocks: call it from a worker, not from the relay executor
        self.commands = queue.Queue()
        threading.Thread(target=self._send_commands, daemon=True).start()
        node.get_logger().info("revo2 relay: on")

    @staticmethod
    def _has_hands(client):
        try:
            return bool(json.loads(client.loco_client.GetHands().to_json_str()).get("body", {}).get("hands"))
        except Exception:
            return False

    def _on_command(self, msg):
        try:
            command = json.loads(msg.data)
            side = command["side"]
            closure = {dof: min(1.0, max(0.0, float(value))) for dof, value in command["closure"].items()}
            if side not in self.targets or not set(closure) <= set(DOFS):
                raise ValueError(f"side must be left/right, dofs one of {DOFS}")
            with self.lock:
                self.targets[side].update(closure)
                target = dict(self.targets[side])
        except Exception as error:
            self.node.get_logger().error(f"revo2: bad hand_cmd {msg.data!r}: {error}")
            return
        self.commands.put((side, target))

    def _send_commands(self):
        while True:
            side, target = self.commands.get()
            params = [self.b.DexterousFingerParameter(
                SEQ[dof], round(ANGLE_OPEN + value * (ANGLE_CLOSED - ANGLE_OPEN)), FORCE, SPEED)
                for dof, value in target.items()]
            try:
                self.loco.ControlDexterousHand(params, self.hand_index[side], self.b.BoosterHandType.kRevoHand)
            except Exception as error:
                self.node.get_logger().error(f"revo2: ControlDexterousHand failed: {error}")

    def _on_hand_data(self, msg):
        side = "left" if int(msg.hand_index) == int(self.hand_index["left"]) else "right"
        angles = {int(p.seq): int(p.angle) for p in msg.hand_data}
        closure = {dof: min(1.0, max(0.0, (angles[seq] - ANGLE_OPEN) / (ANGLE_CLOSED - ANGLE_OPEN))) + 0.0
                   for dof, seq in SEQ.items() if seq in angles}
        with self.lock:
            self.closure[side] = closure

    def _publish_state(self):
        now = round(self.node.get_clock().now().nanoseconds * 1e-9, 3)
        with self.lock:
            states = dict(self.closure)
        for side, closure in states.items():
            msg = String()
            msg.data = json.dumps({"side": side, "closure": closure, "time": now})
            self.publisher.publish(msg)
