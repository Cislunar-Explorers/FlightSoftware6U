from fsw.utils.parameters import HIGH_CRACKING_PRESSURE
from fsw.telemetry.sensor import SynchronousSensor


class DummySynchronousSensor(SynchronousSensor):
    def set_poll_return_value(self, val):
        self.return_val = val


class PressureSensor(SynchronousSensor):
    def __init__(self):
        self.return_val = 0.0
        self.ticker = 0

    # Update value and return it (should also store value in db on poll)
    def poll(self):
        self.ticker += 1
        if self.ticker <= 10:
            self.return_val += HIGH_CRACKING_PRESSURE / 10.0
        if self.ticker >= 15:
            self.ticker = 0
            self.return_val = 0.0
        return self.return_val

    # Return most recently read value
    def read_value(self):
        return self.return_val
