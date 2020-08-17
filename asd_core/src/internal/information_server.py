# This file will track the state of autonomous operations.
# Sector information, path status, ray tracing status, Ray status
# TODO: TCP Server can be implemented


class InformationServer:

    state = {
        "information_server": {"status": "offline"},
        "sector_service": {"status": "offline"},
    }

    def __init__(self):
        super().__init__()

    def is_online(self):
        return True

    def update(self, payload):
        if isinstance(payload, dict):
            self.state.update(payload)
        else:
            raise Exception("Update payload must be a dictionary")

    def get_state(self):
        return self.state

