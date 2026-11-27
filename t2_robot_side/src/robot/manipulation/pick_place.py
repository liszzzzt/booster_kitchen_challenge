"""Side power grasp of upright cylinders (cups, bottles) with the legs locked, for manipulation_command.

    PickPlace(client).execute(command)        command dicts (manipulation_command "type"):
      {"type": "manip_start", "head_pitch": 0.35, "leg_gain_scale": 3.0}   lock the legs (standing), look down
      {"type": "pick",  "side": "right", "coordinate": [x, y, z], "radius": 0.035, "pitch": 25}
            pitch [deg]: hand turned in its palm plane, fingers pointing down, thumb leaning forward
            (thumb and index then reach a cup whose top is below the hand), default 0
            roll [deg]: hand turned about the forward axis, palm facing down-inwards, so thumb and index
            close over the object from above-side (for objects lower than the hand), default 0
        The approach stops at the first contact (sim contact report), then the fingers close.
      {"type": "place", "side": "right", "coordinate": [x, y, z]}
      {"type": "manip_release"}                         arms back to the start pose, Booster takes over
    coordinate: centre of the cylinder at grasp height, in the trunk frame (x forward, y left, z up;
    the server converts camera / odom points with TF). place: where that centre should end up.

Motion (all in the trunk frame, hand orientation "side": fingers forward, palm towards the body midline,
thumb up; the grasp point is <side>_power_grasp_link of revo2_tf, the cylinder axis = its x axis):
    pick : 1. arm up beside the body to above the table top (RAISE)  2. palm turned inwards, straight
           forward above the table  3. straight down beside the cup, palm centre at the cup's centre height
           4. towards the cup axis, posture kept, until the palm is GRASP_GAP from the cup (or touches it)
           5. close: hand shapes tried in turn, each tested with a 2 cm lift; none holds -> open, FAIL
           6. lift 8 cm
    place: straight to above the target -> down until the cup stands on the table (the hand stops
           carrying it) -> open -> palm away 8 cm -> up 5 cm
    release: back down the RAISE way, arms home, legs back to Booster
Safety: the trunk tilt (IMU) is checked during every motion; more than FALL_DEG away from the tilt at
manip_start -> stop sending, FAIL (no active balance with locked legs: a fall cannot be caught).
Legs: ArmController(legs="lock"), see arm_controller.py: no stepping / sway, no active balance either.
The legs stay locked from manip_start (or the first pick) until manip_release, so a held object is
never dropped by a mode change; T2Robot refuses locomotion meanwhile.
Reach of the cylinder centre incl. approach and lift (IK; left = y mirrored): x 0.35..0.45 m,
y -0.14..0.00 m (right hand: the cup in front of the body, the hand comes from its right), z -0.15..-0.05 m,
trunk frame. Out of reach -> IKError before anything moves.
Fingers: closure commands on /revo2/hand_cmd (Studio plugin) and /t2/revo2/hand_cmd (real robot,
Revo2Relay); whichever exists acts.
"""
import json
import logging
import time

import numpy as np

from .arm_controller import IKError, min_jerk, shared_controller
from .arm_ik import joint_names, orientation
from .ik_worker import IKError as WorkerIKError, IKWorker
from .ros_log import route

route("pick_place", "arm_controller")

log = logging.getLogger("pick_place")

# power grasp point in <side>_wrist_roll_link (right; left = y mirrored), as revo2_tf's power_grasp_link:
# palm surface z = -0.009 at y = -0.125, cylinder centre one radius further out
PALM = (0.0, -0.125, -0.009)
APPROACH = 0.08          # pre-grasp / retreat distance along the palm normal [m]
LIFT = 0.08              # lift after grasping [m]
GRASP_GAP = 0.008        # palm surface to cup surface when the hand closes [m]
GRASP_DOWN = 0.007       # palm point this much below the cup centre (along the thumb axis; 1 cm let the pinky
                         # edge of the hand drag on the table and push the cup over) and
GRASP_BACK = 0.005       # this much back towards the wrist: offline MuJoCo sweep (hand + Studio cup mesh,
                         # plugin finger control) grasps reliably within ~+-0.5 cm of this; 1.5 cm higher only the
                         # thumb touches (the lid is wider than the body), 1.5 cm forward the fingers tip the cup over
CLEAR = 0.10             # the hand comes down onto the pre-grasp from this much higher [m]
# Raising the arm when the robot stands close to a table: straight from the hanging pose to the pre-grasp
# the hand would sweep forward below the table top and catch on its edge. So it first goes out to the side
# (still low, not further forward than it hangs, not backwards either: with locked legs any fore-aft swing
# of the arm moves the centre of mass and can tip the robot), up beside the body, and only then forward
# above the table (right hand, trunk frame; left: y mirrored). The same way back down in manip_release.
RAISE = [(0.02, -0.28, -0.22),    # a little back and out (finger tips stay behind x 0.16: the table edge
         (0.08, -0.28, 0.00)]     # must be further away than that), then up beside the body
RAISE_TIME = 3.5         # seconds per RAISE step (slow: no swing)
LOWER_TIME = 4.0         # seconds per step on the way down (manip_release), slower still
ARM_GAIN_SCALE = 4.0     # arm kp/kd x this: with the example gains (x1) the outstretched arm sags ~7 cm
                         # under gravity, the hand lands on the table instead of next to the cup
CUP_AXIS_IN_HAND = (0.985, -0.14, -0.09)   # cup axis in the wrist frame once held (offline sim: the power grip
                         # leaves the cup ~10 deg tilted, top towards the finger tips); placed like that it lands on
                         # an edge and tips over when the hand opens -> place() turns the hand to stand it upright
STANCE_PITCH = 6.0       # trunk pitch [deg] the locked stance is levelled to (stable in every run so far)
FALL_DEG = 12.0          # trunk tilt change (IMU) that counts as falling [deg]
SPEED = 0.06             # straight-line speed near the object [m/s]
# open for the approach: thumb NOT rotated in (rotated it sticks out over the palm by ~8 cm and hits the
# object first), it swings in only when closing
HAND_OPEN = dict(thumb_rotation=0.0, thumb_flex=0.0, index=0.0, middle=0.0, ring=0.0, pinky=0.0)
HAND_WRAP = dict(thumb_rotation=0.0, thumb_flex=0.0, index=1.0, middle=1.0, ring=1.0, pinky=1.0)   # fingers first
HAND_POWER = dict(thumb_rotation=0.9, thumb_flex=0.75, index=1.0, middle=1.0, ring=1.0, pinky=1.0)
CLOSE_TIME = 1.5         # wait for the fingers [s]
# hand shapes tried in turn by pick(..., shapes=[...]) until one holds the object when lifted
HAND_SHAPES = {
    "power":      dict(thumb_rotation=0.9, thumb_flex=0.75, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
    "power_firm": dict(thumb_rotation=1.0, thumb_flex=1.0, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
    "wrap_soft":  dict(thumb_rotation=0.8, thumb_flex=0.6, index=0.8, middle=0.85, ring=0.9, pinky=0.9),
    "thumb_wide": dict(thumb_rotation=0.6, thumb_flex=1.0, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
}
TEST_LIFT = 0.02         # short lift that tells whether a hand shape holds [m]
HAND_TOPICS = ("/revo2/hand_cmd", "/t2/revo2/hand_cmd")
CONTACT_TOPIC = "/revo2/contacts"                         # Studio plugin: per-hand contact report
STATE_TOPICS = ("/revo2/hand_state", "/t2/revo2/hand_state")
IMU_TOPICS = ("/imu/data", "/t2/imu/data")
MIN_GRIP_N = 0.5         # grasp check: normal force on the object [N]


class GraspError(RuntimeError):
    pass


class FallError(RuntimeError):
    pass


class Hands:
    """Finger commands for both backends (Studio plugin / real Revo2Relay) plus what is needed to check
    the grasp and the balance: contact report (sim), finger state, IMU. Own node, spun in its own thread."""

    def __init__(self):
        import threading
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        self._String = String
        self.node = Node("t2_pick_place_hands")
        self.publishers = [self.node.create_publisher(String, t, 10) for t in HAND_TOPICS]
        self.contacts, self.state, self.imu = {}, {}, None     # side -> (receive time, message)
        self.node.create_subscription(String, CONTACT_TOPIC, self._on_contacts, 10)
        for t in STATE_TOPICS:
            self.node.create_subscription(String, t, self._on_state, 10)
        for t in IMU_TOPICS:
            self.node.create_subscription(Imu, t, self._on_imu, qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def set(self, side, closure):
        msg = self._String()
        msg.data = json.dumps({"side": side, "closure": closure})
        for p in self.publishers:
            p.publish(msg)

    # ---------------------------------------------------------------- feedback
    def _on_contacts(self, msg):
        try:
            m = json.loads(msg.data)
            self.contacts[m["side"]] = (time.monotonic(), m)
        except Exception:
            pass

    def _on_state(self, msg):
        try:
            m = json.loads(msg.data)
            self.state[m["side"]] = (time.monotonic(), m)
        except Exception:
            pass

    def _on_imu(self, msg):
        self.imu = (time.monotonic(), msg.orientation)

    def tilt_deg(self):
        """(roll, pitch) of the trunk in degrees, or None without a recent IMU message."""
        if self.imu is None or time.monotonic() - self.imu[0] > 1.0:
            return None
        q = self.imu[1]
        roll = np.degrees(np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        pitch = np.degrees(np.arcsin(np.clip(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)))
        return float(roll), float(pitch)

    def load(self, side):
        """Upward force [N] the hand puts on the objects it touches (not the table), sim contact report."""
        c = self.contacts.get(side)
        if c is None or time.monotonic() - c[0] > 1.0:
            return None
        return float(sum(o.get("net_force", [0, 0, 0])[2] for n, o in c[1].get("objects", {}).items()
                         if "table" not in n.lower()))

    def touching(self, side):
        """The palm touching something that is not the table (sim contact report). Finger tips / thumb
        touching first must not stop the approach: the object would then sit outside the hand."""
        c = self.contacts.get(side)
        if c is None or time.monotonic() - c[0] > 1.0:
            return False
        objects = [n for n, o in c[1].get("objects", {}).items() if "table" not in n.lower() and o.get("fn", 0) > 0.2]
        palm = c[1].get("fingers", {}).get("palm", {}).get("n", 0) > 0
        return bool(objects) and palm

    def detail(self, side):
        """Short text of the last contact report: per finger n/fn, per object fn (sim only)."""
        c = self.contacts.get(side)
        if c is None:
            return "no contact report"
        m = c[1]
        f = " ".join(f"{k}:{v.get('n', 0)}/{v.get('fn', 0):.2f}" for k, v in m.get("fingers", {}).items())
        o = " ".join(f"{k}:{v.get('fn', 0):.2f}" for k, v in m.get("objects", {}).items())
        return f"fingers[{f}] objects[{o}] age {time.monotonic() - c[0]:.1f}s"

    def held(self, side):
        """(held, reason): True / False, or None when there is no feedback to decide with."""
        c = self.contacts.get(side)
        if c is not None and time.monotonic() - c[0] < 1.0:
            m = c[1]
            fingers = m.get("fingers", {})
            thumb = fingers.get("thumb", {}).get("n", 0) > 0
            other = any(fingers.get(k, {}).get("n", 0) > 0 for k in ("index", "middle", "ring", "pinky", "palm"))
            objects = {n: o for n, o in m.get("objects", {}).items()
                       if "table" not in n.lower() and o.get("fn", 0.0) >= MIN_GRIP_N}
            if thumb and other and objects:
                return True, f"holding {sorted(objects)} (grip {m.get('grip_fn', 0):.1f} N)"
            return False, f"no grip (thumb contact {thumb}, objects {sorted(m.get('objects', {}))})"
        st = self.state.get(side)                       # real hand: closed fingers stopped early = object
        if st is not None and time.monotonic() - st[0] < 1.0:
            cl = st[1].get("closure", {})
            blocked = [k for k in ("index", "middle") if k in cl and cl[k] < 0.85]
            return (bool(blocked), f"finger closure {cl}")
        return None, "no hand feedback"


class PickPlace:
    def __init__(self, client, max_vel=0.5):
        self.client = client
        self.max_vel = max_vel
        self.arm = None
        self.hands = None
        self.home = {}
        self.holding = {}            # side -> radius of the held cylinder
        self.cart = {"left": np.zeros(3), "right": np.zeros(3)}   # Cartesian correction of the grasp point
        self.raised = set()          # sides whose arm went up the safe way (RAISE) and must come down that way
        self.tilt0 = None            # trunk (roll, pitch) right after locking the legs [deg]
        self.offset = {"left": np.zeros(7), "right": np.zeros(7)}   # sag compensation added to the commands
        self.ik = IKWorker()         # IK in its own process (the agent's relay threads starve it here)

    @property
    def active(self):
        return self.arm is not None and self.arm.enabled

    # ------------------------------------------------------------ commands
    def execute(self, command):
        kind = command.get("type")
        if kind == "manip_start":
            self.start(float(command.get("head_pitch", 0.35)), float(command.get("leg_gain_scale", 3.0)))
        elif kind == "pick":
            self.pick(command["side"], command["coordinate"], float(command.get("radius", 0.035)),
                      float(command.get("pitch", 0.0)), float(command.get("roll", 0.0)),
                      command.get("shapes"))
        elif kind == "place":
            self.place(command["side"], command["coordinate"])
        elif kind == "manip_release":
            self.release()
        else:
            raise ValueError(f"unknown manipulation type {kind!r}")
        return True

    def start(self, head_pitch=0.35, leg_gain_scale=3.0):
        if self.hands is None:
            self.hands = Hands()
        # one controller for the whole agent (arm_teleop's arm_jog uses the same one)
        self.arm = shared_controller(self.client, legs="lock", max_vel=self.max_vel, leg_gain_scale=leg_gain_scale,
                                     gain_scale=ARM_GAIN_SCALE)
        if not self.arm.enabled:
            self.arm.start()
            self.home = dict(self.arm.target)
        self.arm.move_joints({"head_pitch_joint": head_pitch, "aa_head_yaw_joint": 0.0}, duration=1.5)
        time.sleep(0.5)
        self._level_stance()
        self.tilt0 = self.hands.tilt_deg()
        log.info("legs locked, trunk tilt %s deg", None if self.tilt0 is None else np.round(self.tilt0, 1))

    def _level_stance(self):
        """The prepare posture the legs get locked in varies (trunk pitch 1.4..12 deg): leaning back
        (< ~4 deg) the robot tips over backwards once the arm swings out, leaning forward (> ~9 deg) it
        is too close / tips forwards. Both ankles pitch together, in small steps, until the trunk pitch
        (IMU) is STANCE_PITCH +- 1 deg. Measured: ankle pitch -1.3 deg -> trunk pitch +2.2 deg. Stops when
        the error grows (never flips direction: a wrong reading must not run away), at most 5 deg."""
        names = ("left_ankle_pitch_joint", "right_ankle_pitch_joint")
        if not all(n in self.arm.target for n in names):
            return
        start = {n: self.arm.target[n] for n in names}
        time.sleep(1.0)                                # let the sway after the lock die out
        t = self.hands.tilt_deg()
        if t is None:
            return
        err0 = STANCE_PITCH - t[1]
        if abs(err0) > 10.0:
            log.warning("stance: trunk pitch %.1f deg, not levelling", t[1])
            return
        for _ in range(6):
            t = self.hands.tilt_deg()
            if t is None:
                break
            err = STANCE_PITCH - t[1]
            if abs(err) < 1.0 or abs(err) > abs(err0) + 1.0:
                break
            step = float(np.clip(err, -1.5, 1.5))
            goal = {n: self.arm.target[n] - np.radians(step) for n in names}   # ankle - -> trunk leans forward
            if any(abs(goal[n] - start[n]) > np.radians(5.0) for n in names):
                break
            self.arm.move_joints(goal, duration=1.0)
            time.sleep(1.2)
            t2 = self.hands.tilt_deg()
            log.info("  stance step: ankles %+.1f deg -> trunk pitch %.1f -> %s", -step, t[1],
                     None if t2 is None else round(t2[1], 1))
        t = self.hands.tilt_deg()
        log.info("stance: trunk pitch %.1f deg (target %.1f), ankles %+.1f deg", t[1] if t else float("nan"),
                 STANCE_PITCH, np.degrees(self.arm.target[names[0]] - start[names[0]]))

    def pick(self, side, centre, radius=0.035, pitch=0.0, roll=0.0, shapes=None):
        self.start_if_needed()
        if side in self.holding:
            raise RuntimeError(f"{side} hand already holds an object")
        point, R, normal = self._frame(side, radius, pitch, roll)
        centre = np.asarray(centre, float)
        centre = centre - GRASP_DOWN * R[:, 0] + GRASP_BACK * R[:, 1]   # R[:,0] thumb (up), -R[:,1] fingers
        log.info("pick %s: open hand; palm target %s", side, np.round(centre, 3))
        self.hands.set(side, HAND_OPEN)
        pre = centre - APPROACH * normal              # beside the cup, palm facing it, at its centre height
        turn = self._raise_points(side)[-1]           # beside the body above table height
        self._check(side, point, R, [turn, pre + [0, 0, CLEAR], pre, centre, centre + [0, 0, LIFT]])
        log.info("pick %s: waypoints ok, pre-grasp %s", side, np.round(pre, 3))
        # 1. arm up beside the body, above the table top
        self._raise(side, point)
        # 2. palm turned inwards (towards the cup), then straight forward above the table
        self._move(side, point, turn, R)
        log.info("pick %s: above the table, palm inwards; forward", side)
        # joint move, not a straight line: the straight hand path from beside the body to over the table
        # passes poses the arm cannot take with the palm inwards; both ends are well above the table top
        self.cart[side] = np.zeros(3)
        self._move(side, point, pre + [0, 0, CLEAR], R, RAISE_TIME)
        self._compensate(side)
        self._settle(side, point, pre + [0, 0, CLEAR], R)
        # 3. straight down beside the cup: palm centre at the cup's centre height
        self._line(side, point, pre, R)
        self._compensate(side)
        self._settle(side, point, pre, R)
        log.info("pick %s: beside the cup (%s)", side, self._palm_gap(side, radius, centre))
        # 4. towards the cup axis, posture kept, until the palm is one radius from the axis (or touches)
        self._line(side, point, centre, R, stop=lambda: self.hands.touching(side))
        # palm as close to the cup wall as possible: on, 4 mm at a time, until the palm itself reports
        # contact with the cup (the geometric gap alone left ~1 cm and the fingers closed in front of it)
        for _ in range(8):                            # up to 3.2 cm past the geometric target
            if self.hands.touching(side):
                break
            here = self._fk(side, point)
            self._line(side, point, here + 0.004 * normal, R, stop=lambda: self.hands.touching(side))
        log.info("pick %s: palm contact %s | %s", side, self.hands.touching(side), self.hands.detail(side))
        log.info("pick %s: at the cup (%s)", side, self._palm_gap(side, radius, centre))
        held, why, used = False, "", None
        for name in (shapes or ["power"]):
            shape = HAND_SHAPES[name]
            self.hands.set(side, dict(HAND_WRAP, **{k: shape[k] for k in ("index", "middle", "ring", "pinky")}))
            time.sleep(0.8)                                      # fingers around the object ...
            self.hands.set(side, dict(shape, thumb_flex=0.0))    # ... thumb swings in ...
            time.sleep(0.6)
            self.hands.set(side, shape)                          # ... and presses
            time.sleep(CLOSE_TIME)
            log.info("pick %s: shape %s closed: %s | %s", side, name, self.hands.held(side)[1],
                     self.hands.detail(side))
            here = self._fk(side, point)
            self._line(side, point, here + [0, 0, TEST_LIFT], R)            # does it come along?
            time.sleep(0.5)
            held, why = self.hands.held(side)
            log.info("pick %s: shape %s after %.0f cm lift: %s", side, name, 100 * TEST_LIFT, why)
            if held is not False:
                used = name
                break
            self._line(side, point, here, R)                                # back down, open, next shape
            self.hands.set(side, HAND_OPEN)
            time.sleep(1.0)
        if held is False:
            raise GraspError(f"{side} hand did not get the object with any of {shapes or ['power']} ({why}); "
                             f"nothing done after the pick")
        log.info("pick %s: holding with shape %s, lift", side, used)
        self._line(side, point, centre + [0, 0, LIFT], R)
        time.sleep(0.5)
        held, why = self.hands.held(side)
        log.info("pick %s: grasp check after lift: %s", side, why)
        if held is False:
            self.hands.set(side, HAND_OPEN)
            raise GraspError(f"{side} hand lost the object while lifting ({why}); nothing done after the pick")
        self.holding[side] = (radius, pitch, roll)
        log.info("picked with %s hand at %s", side, np.round(centre, 3))

    def place(self, side, centre):
        self.start_if_needed()
        if side not in self.holding:
            raise RuntimeError(f"{side} hand holds nothing")
        point, R, normal = self._frame(side, *self.holding[side])
        centre = np.asarray(centre, float)
        centre = centre - GRASP_DOWN * R[:, 0] + GRASP_BACK * R[:, 1]   # same hand-to-cup offset as in pick
        above = centre + [0, 0, LIFT]
        self._check(side, point, R, [above])
        R0 = R
        fracs = [f for f in (1.0, 0.8, 0.6, 0.4, 0.2, 0.0)]   # cup turned upright in the hand, as far as
        while True:                                            # the arm reaches (the end points: _check)
            R = self._upright(side, R0, fracs[0])
            normal = -R[:, 2]
            out = centre - APPROACH * normal
            try:
                self._check(side, point, R, [above, centre - [0, 0, 0.015]])   # (retreat has a fallback)
                break
            except (IKError, WorkerIKError):
                if len(fracs) == 1:
                    raise
                fracs.pop(0)
        load = self.hands.load(side)                  # how much the hand carries now (cup weight)
        log.info("place %s: to above the target (carrying %s N)", side, None if load is None else round(load, 2))
        self._line(side, point, above, R0)            # straight, at lift height
        # down until the cup stands on the table: the hand stops carrying it (or 3 cm below the target)
        # (the load reading jumps while the arm moves: count it only within 1.5 cm of the target height
        # and only when it stays low for two checks in a row)
        low = [0]
        def landed():
            now = self.hands.load(side)
            near = self._fk(side, point)[2] < centre[2] + 0.015
            low[0] = low[0] + 1 if (near and load is not None and load > 0.2 and now is not None
                                    and now < 0.4 * load) else 0
            return low[0] >= 2
        while True:                                   # turn the hand (cup upright), then straight down; if that
            R = self._upright(side, R0, fracs[0])     # straight line is not feasible, a little less upright
            normal = -R[:, 2]
            self._move(side, point, above, R, 1.5)
            log.info("place %s: cup turned upright by %.0f%%, down", side, 100 * fracs[0])
            try:
                self._line(side, point, centre - [0, 0, 0.015], R, stop=landed)
                break
            except (IKError, WorkerIKError) as error:
                if len(fracs) == 1:
                    raise
                log.info("place %s: %s; less upright", side, error)
                fracs.pop(0)
        time.sleep(0.3)
        log.info("place %s: cup on the table (carrying %s N), open", side, self.hands.load(side))
        self.hands.set(side, HAND_OPEN)
        time.sleep(CLOSE_TIME)
        here = self._fk(side, point)
        try:
            self._line(side, point, here - APPROACH * normal, R)          # palm away from the cup ...
            self._line(side, point, here - APPROACH * normal + [0, 0, 0.05], R)   # ... and up
        except (IKError, WorkerIKError) as error:
            log.info("place %s: retreat %s; away 3 cm and up instead", side, error)
            here = self._fk(side, point)
            self._line(side, point, here - 0.03 * normal, R)
            self._line(side, point, here - 0.03 * normal + [0, 0, 0.06], R)
        del self.holding[side]
        log.info("placed with %s hand at %s", side, np.round(centre, 3))

    def release(self):
        if self.holding and self._tilt_ok():
            raise RuntimeError(f"still holding with {sorted(self.holding)}: place first")
        self.holding.clear()
        if self.active:
            try:
                for side in sorted(self.raised):       # arms all the way down first ...
                    self._lower(side)
                self._home(LOWER_TIME)                 # ... to the start posture ...
                time.sleep(1.0)                        # ... settle, then Booster takes over (stop below)
                log.info("release: arms down, trunk tilt %s deg; legs back to Booster", self.hands.tilt_deg())
            except FallError as error:
                log.error("release: %s; legs handed back to Booster without moving the arms", error)
            finally:
                self.raised.clear()
                self.arm.stop()

    def start_if_needed(self):
        if not self.active:
            self.start()

    # ------------------------------------------------------------ helpers
    def _frame(self, side, radius, pitch=0.0, roll=0.0):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        mirror = 1.0 if side == "right" else -1.0                 # PALM is the right hand's
        point = (PALM[0], mirror * PALM[1], PALM[2] - radius)     # cylinder centre in the wrist frame
        a = np.radians(pitch)                                     # hand turned in its palm plane:
        Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])  # fingers down,
        b = np.radians(roll) * mirror                             # turned about the forward axis: palm
        Rx = np.array([[1, 0, 0], [0, np.cos(-b), -np.sin(-b)], [0, np.sin(-b), np.cos(-b)]])  # faces down-in,
        R = Ry @ Rx @ orientation(side, "side")                   # thumb / index come over the object
        normal = -R[:, 2]                                         # palm normal, towards the object
        return point, R, normal

    def _raise_points(self, side):
        mirror = 1.0 if side == "right" else -1.0
        return [np.array([x, mirror * y, z]) for x, y, z in RAISE]

    def _raise(self, side, point):
        """Arm up the table-safe way: out to the side, up beside the body (any hand orientation)."""
        if side in self.raised:
            return
        for p in self._raise_points(side):
            q, info = self.ik.solve(side, point, p, None, self._desired(side))
            if not info["ok"]:
                raise IKError(f"{side}: raise waypoint {np.round(p, 3)} not reachable")
            self._joint_move(side, q, RAISE_TIME)
        self.raised.add(side)

    def _lower(self, side):
        """Reverse of _raise, slowly: beside the body above table height, out to the side and down."""
        point = self._frame(side, 0.035)[0]
        for p in reversed(self._raise_points(side)):
            q, info = self.ik.solve(side, point, p, None, self._desired(side))
            if info["ok"]:
                self._joint_move(side, q, LOWER_TIME)
        self.raised.discard(side)

    def _check(self, side, point, R, points):
        """Solve every waypoint before moving (no half-finished motion)."""
        q = self._desired(side)
        for p in points:
            q, info = self.ik.solve(side, point, p, R, q)
            if not info["ok"]:
                raise IKError(f"{side}: {np.round(p, 3)} out of reach "
                              f"(reach: x 0.35..0.45, |y| 0.00..0.14 on its side, z -0.15..-0.05 m, trunk frame)")

    def _move(self, side, point, pos, R, duration=None):
        """Free-space joint move to an IK solution (curved hand path)."""
        q, info = self.ik.solve(side, point, np.asarray(pos) + self.cart[side], R, self._desired(side))
        if not info["ok"]:
            raise IKError(f"{side}: {np.round(pos, 3)} not reachable")
        self._joint_move(side, q, duration)

    def _line(self, side, point, pos, R, stop=None):
        """Straight-line move of the grasp point at SPEED (path solved first, then executed)."""
        path = self.ik.line(side, point, self._desired(side), np.asarray(pos) + self.cart[side], R, SPEED, self.arm.dt)
        names = joint_names(side)
        t0 = time.monotonic()
        for k, q in enumerate(path, 1):
            if k % 5 == 0:
                self._guard()
            if stop is not None and k % 3 == 0 and stop():
                log.info("  stopped at contact after %d of %d steps", k, len(path))
                break
            self.arm.target.update(zip(names, map(float, q + self.offset[side])))
            self.arm._send()
            time.sleep(max(0.0, t0 + k * self.arm.dt - time.monotonic()))
        self._guard()

    def _joint_move(self, side, q, duration=None):
        """ArmController.move_arm_q in short pieces, checking the balance in between."""
        start = self._desired(side)
        q = np.asarray(q, float)
        dmax = float(np.max(np.abs(q - start)))
        T = max(duration or 0.0, dmax / self.arm.max_vel * 1.875, 0.2)
        names = joint_names(side)
        steps = max(1, int(round(T / self.arm.dt)))
        t0 = time.monotonic()
        for k in range(1, steps + 1):
            if k % 5 == 0:
                self._guard()
            a = min_jerk(k / steps)
            self.arm.target.update(zip(names, map(float, start + a * (q - start) + self.offset[side])))
            self.arm._send()
            time.sleep(max(0.0, t0 + k * self.arm.dt - time.monotonic()))
        self._guard()

    def _home(self, duration):
        """All upper-body joints back to the posture of manip_start (sag compensation removed)."""
        for side in ("left", "right"):
            self.offset[side] = np.zeros(7)
            self.cart[side] = np.zeros(3)
        start = dict(self.arm.target)
        steps = max(1, int(round(duration / self.arm.dt)))
        t0 = time.monotonic()
        for k in range(1, steps + 1):
            if k % 5 == 0:
                self._guard()
            a = min_jerk(k / steps)
            for n, v in self.home.items():
                self.arm.target[n] = start[n] + a * (v - start[n])
            self.arm._send()
            time.sleep(max(0.0, t0 + k * self.arm.dt - time.monotonic()))

    def _desired(self, side):
        """Arm joint angles we want (the command minus the sag compensation)."""
        return self.arm.arm_q(side, measured=False) - self.offset[side]

    def _compensate(self, side, rounds=3):
        """Hold still and add the remaining joint error (gravity sag) to the command, a few times."""
        names = joint_names(side)
        q = self._desired(side)
        for _ in range(rounds):
            time.sleep(0.4)
            err = q - self.arm.arm_q(side, measured=True)
            self.offset[side] = np.clip(self.offset[side] + err, -0.3, 0.3)
            self.arm.target.update(zip(names, map(float, q + self.offset[side])))
            self.arm._send()
        time.sleep(0.4)
        self._guard()

    def _settle(self, side, point, pos, R, tol=0.003, rounds=6):
        """Closed loop on the grasp point: measured joints -> FK -> remaining error added to a Cartesian
        correction (kept for the following moves), until the point is within tol of pos."""
        from .arm_ik import ArmIK
        fk = ArmIK(side, point=point).fk
        pos = np.asarray(pos, float)
        e = None
        for i in range(rounds):
            time.sleep(0.5)
            e = pos - fk(self.arm.arm_q(side, measured=True))[0]
            if np.linalg.norm(e) < tol:
                break
            self.cart[side] = np.clip(self.cart[side] + e, -0.05, 0.05)
            q, info = self.ik.solve(side, point, pos + self.cart[side], R, self._desired(side))
            if not info["ok"]:
                log.warning("settle %s: corrected target %s not reachable", side, np.round(pos + self.cart[side], 3))
                break
            self._joint_move(side, q, 0.6)
        log.info("settle %s: error %s mm, correction %s mm", side, np.round(1000 * e, 1), np.round(1000 * self.cart[side], 1))
        self._guard()

    def _upright(self, side, R, frac=1.0):
        """Hand orientation that makes the held cup's axis vertical (gravity from the IMU)."""
        a = np.array(CUP_AXIS_IN_HAND, float)
        if side == "left":
            a[1] = -a[1]
        a = R @ (a / np.linalg.norm(a))                       # cup axis in the trunk frame now
        t = self.hands.tilt_deg() or (0.0, 0.0)
        r, p = np.radians(t)
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        up = (Ry @ Rx).T @ np.array([0.0, 0.0, 1.0])          # gravity up in the trunk frame
        v = np.cross(a, up)
        s_ = float(np.linalg.norm(v))
        if s_ < 1e-9:
            return R
        ang = np.arctan2(s_, float(a @ up)) * frac
        k = v / s_
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return (np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K) @ R

    def _fk(self, side, point):
        from .arm_ik import ArmIK
        return ArmIK(side, point=point).fk(self._desired(side))[0] - self.cart[side]

    def _gap(self, side, radius, centre):
        """Measured palm surface to cylinder surface distance [m] (horizontal, cylinder axis vertical)."""
        try:
            from .arm_ik import ArmIK
            mirror = 1.0 if side == "right" else -1.0
            palm = ArmIK(side, point=(PALM[0], mirror * PALM[1], PALM[2])).fk(self.arm.arm_q(side, measured=True))[0]
            d = palm - np.asarray(centre, float)
            return float(np.hypot(d[0], d[1])) - radius
        except Exception:
            return None

    def _palm_gap(self, side, radius, centre):
        """Where the palm really is relative to the cylinder: distance from the palm centre (measured
        joints) to the cylinder axis, perpendicular to the axis, minus the radius = the gap to close."""
        from .arm_ik import ArmIK
        mirror = 1.0 if side == "right" else -1.0
        palm = ArmIK(side, point=(PALM[0], mirror * PALM[1], PALM[2])).fk(self.arm.arm_q(side, measured=True))[0]
        d = palm - np.asarray(centre, float)
        d_axis = float(np.hypot(d[0], d[1]))                 # cylinder axis vertical (trunk z)
        gap = d_axis - radius
        ok = abs(gap) < 0.01 and abs(d[2]) < 0.04
        return (f"palm centre {np.round(palm, 3)}: {100 * d_axis:.1f} cm from the cup axis, gap {100 * gap:+.1f} cm "
                f"(radius {100 * radius:.1f}), {100 * d[2]:+.1f} cm above the target height -> "
                f"{'graspable' if ok else 'NOT in grasp position'}")

    def _tracking(self, side, point):
        """Commanded vs measured grasp point (arm sag under gravity / contact), for the log."""
        try:
            from .arm_ik import ArmIK
            ik = ArmIK(side, point=point)
            pc = ik.fk(self._desired(side))[0]
            pm = ik.fk(self.arm.arm_q(side, measured=True))[0]
            return f"commanded {np.round(pc, 3)}, measured {np.round(pm, 3)}"
        except Exception as error:
            return f"tracking n/a: {error}"

    def _tilt_ok(self):
        try:
            self._guard()
            return True
        except FallError:
            return False

    def _guard(self):
        """Stop everything if the trunk tilts away (falling): FallError, no further commands."""
        t = self.hands.tilt_deg() if self.hands else None
        if t is None or self.tilt0 is None:
            return
        d = max(abs(t[0] - self.tilt0[0]), abs(t[1] - self.tilt0[1]))
        if d > FALL_DEG:
            raise FallError(f"trunk tilt {np.round(t, 1)} deg, {d:.0f} deg away from the start: falling, stopped")
