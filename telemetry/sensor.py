from threading import Thread
from time import time

# What functionality should my sensor have?
# Does it need to maintain its own state or should it take inputs?
# Does it need control over its own thread when executed?

# Minimally: it should initialize itself and be capable of polling
# the physical sensor to update all of its fields
# Each field should store the information about when it was last polled
# it should make the most recent information accessible
class SynchronousSensor:

    # Initialize sensor
    def __init__(self, parent):
        self.parent = parent
        self.poll_time = -1.0

    # poll should poll the sensor and update all of the sensors fields
    def poll(self):
        self.poll_time = time()


class AsynchronousSensor(Thread):
    def __init__(self, name: str, fields: list):
        self.name = name
        self.fields = {field: None for field in fields}

    def poll(self):
        self.run()

    def run(self):
        pass
