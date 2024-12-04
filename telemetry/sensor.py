from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from fsw_main.main import MainSatelliteThread
# for an explanation of the above 4 lines of code, see
# https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
# It lets your IDE know what type(self._main) is, without causing any circular imports at runtime.

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
    def __init__(self, main: MainSatelliteThread):
        # TODO: instead of initializing with main, only use main's driver object for each sensor.
        #  i.e. __init__(self, sensor: GyroSensor); self.sensor = sensor
        self._main = main
        self.poll_time = -1.0
        # self.result = None

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
