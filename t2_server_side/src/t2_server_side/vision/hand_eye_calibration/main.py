"""Head-camera hand-eye calibration from T2WS (Booster's auto-hand-eye service on the robot).

Talks to the robot-side HandEyeCalibRelay (t2_robot_side/src/robot/relay_bridges/handeye_calib.py),
so the T2 agent must be running. Works the same against Booster Studio and the real robot.

    cd /ws/t2_server_side/src/t2_server_side/vision/hand_eye_calibration
    python3 main.py status                    # service there? what is it doing?
    python3 main.py start --square 0.025      # start, print progress until done, print the result
    python3 main.py watch                     # progress of a running job
    python3 main.py result                    # summary, rms_px (reprojection error), full result
    python3 main.py apply                     # write the result to the camera extrinsics (check rms_px first)
    python3 main.py stop

--square: checkerboard square edge length in metres (omit = service default).
Detection view while it runs:  ros2 run rqt_image_view rqt_image_view /t2/handeye_calib/feedback/image
"""
import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DONE = {"done", "finished", "success", "succeeded", "completed", "failed", "error", "stopped", "idle"}
TIMEOUT = 15.0          # one RPC on the robot waits up to ~5 s for the service


class HandEyeCalib(Node):
    def __init__(self):
        super().__init__("t2_handeye_calib")
        self.replies = {}
        self.publisher = self.create_publisher(String, "/t2/handeye_calib/command", 10)
        self.create_subscription(String, "/t2/handeye_calib/reply", self._on_reply, 10)

    def _on_reply(self, msg):
        reply = json.loads(msg.data)
        self.replies[reply.get("id")] = reply

    def call(self, cmd, **params):
        deadline = time.monotonic() + TIMEOUT
        while self.publisher.get_subscription_count() == 0:          # wait for DDS discovery
            if time.monotonic() > deadline:
                raise RuntimeError("no subscriber on /t2/handeye_calib/command: is the T2 agent running?")
            rclpy.spin_once(self, timeout_sec=0.1)
        request_id = uuid.uuid4().hex
        msg = String()
        msg.data = json.dumps({"id": request_id, "cmd": cmd, **params})
        self.publisher.publish(msg)
        while request_id not in self.replies:
            if time.monotonic() > deadline:
                raise RuntimeError(f"{cmd}: no reply within {TIMEOUT:.0f} s")
            rclpy.spin_once(self, timeout_sec=0.1)
        reply = self.replies.pop(request_id)
        if not reply.get("ok"):
            raise RuntimeError(f"{cmd} failed on the robot: {reply.get('error')}")
        return reply.get("data", {})

    def watch(self, period):
        last = None
        while True:
            s = self.call("status")
            line = (f"{s.get('status')}  stage={s.get('stage')}  {100 * s.get('progress', 0):.0f}%  "
                    f"stage2 {s.get('stage2_done')}/{s.get('stage2_total')}  "
                    f"stage3 {s.get('stage3_done')}/{s.get('stage3_total')}")
            if line != last:
                print(time.strftime("%H:%M:%S"), line, flush=True)
                last = line
            if "error" in s:
                print("error:", json.dumps(s["error"], ensure_ascii=False))
            if str(s.get("status", "")).lower() in DONE:
                return s
            time.sleep(period)


def show(data):
    print(json.dumps(data, indent=2, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["status", "start", "watch", "result", "apply", "stop"])
    ap.add_argument("--square", type=float, default=0.0, help="checkerboard square size [m]")
    ap.add_argument("--period", type=float, default=1.0, help="status poll period [s]")
    a = ap.parse_args()

    rclpy.init()
    node = HandEyeCalib()
    try:
        if a.cmd == "start":
            params = {"square_size_m": a.square} if a.square > 0 else {}
            show(node.call("start", **params))
            node.watch(a.period)
            show(node.call("result"))
        elif a.cmd == "watch":
            node.watch(a.period)
        else:
            show(node.call(a.cmd))
    except KeyboardInterrupt:
        print("\ninterrupted (the job keeps running on the robot; 'stop' to cancel it)")
    except RuntimeError as error:
        print(error, file=sys.stderr)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
