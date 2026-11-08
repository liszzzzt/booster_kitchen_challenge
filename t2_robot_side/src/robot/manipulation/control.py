from .remote_control import ManipulationRemoteControl


class ManipulationControl():
    def __init__(self, client):
        self.client = client
        self.remote_control = ManipulationRemoteControl()

    def execute(self, msg):
        return True
