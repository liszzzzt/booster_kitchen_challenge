class Nav2Control:
    MAX_FORWARD_SPEED = 0.4  # m/s
    MAX_ROTATION_SPEED = 0.6  # rad/s

    def __init__(self, client):
        self.client = client

    def start(self):
        self.client.set_mode("walk")

    def execute(self, msg):
        forward = max(
            -self.MAX_FORWARD_SPEED,
            min(self.MAX_FORWARD_SPEED, msg.twist.linear.x),
        )
        rotation = max(
            -self.MAX_ROTATION_SPEED,
            min(self.MAX_ROTATION_SPEED, msg.twist.angular.z),
        )

        self.client.set_velocity(forward, 0.0, rotation)

        return True

    def stop(self):
        self.client.set_velocity(0.0, 0.0, 0.0)

