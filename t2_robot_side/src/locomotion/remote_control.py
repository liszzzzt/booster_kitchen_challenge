import time


class LocomotionRemoteControl():
    def __init__(self, client):
        self.client = client
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.walk_duration = 2.0
        self.head_step = 0.1

    def execute(self, key):
        key = key.lower()

        # walk
        if key in ("w", "s", "a", "d", "q", "e"):
            self.client.set_mode("walk")
            if key == "w":
                v = [self.linear_speed, 0.0, 0.0]
            elif key == "s":
                v = [-self.linear_speed, 0.0, 0.0]
            elif key == "a":
                v = [0.0, self.linear_speed, 0.0]
            elif key == "d":
                v = [0.0, -self.linear_speed, 0.0]
            elif key == "q":
                v = [0.0, 0.0, self.angular_speed]
            else:
                v = [0.0, 0.0, -self.angular_speed]
            return self.walk(v)

        # move_head: T2 feedback uses joint names instead of GR3's head group.
        if key in ("r", "t", "y", "u"):
            self.client.set_mode("walk")
            current = self.client.get_joint_states()
            yaw = current.get_joint("aa_head_yaw_joint").position
            pitch = current.get_joint("head_pitch_joint").position
            if key == "r":
                yaw += self.head_step
            elif key == "t":
                yaw -= self.head_step
            elif key == "y":
                pitch -= self.head_step
            else:
                pitch += self.head_step
            yaw = max(-0.785, min(0.785, yaw))
            pitch = max(-0.3, min(0.85, pitch))
            return self.move_head(yaw, pitch)

        # GR3 stand_pose keys have no matching BoosterOS 1.2.2 API.
        return False

    def walk(self, v):
        # T2 set_velocity has no duration argument; finish the pulse here.
        try:
            self.client.set_velocity(v[0], v[1], v[2])
            time.sleep(self.walk_duration)
        finally:
            self.client.set_velocity(0.0, 0.0, 0.0)
        return True

    def move_head(self, yaw, pitch):
        self.client.set_head_angle(pitch, yaw)
        return True
