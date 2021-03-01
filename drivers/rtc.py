import board
import busio
import adafruit_ds3231
from time import gmtime, clock_settime
from calendar import timegm


class RTC:
    """Interface class for the Adafruit DS3231. This clock only supports second-level precision, but is extremely
    accurate and will run whenever the gom has any power."""

    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ds3231 = adafruit_ds3231.DS3231(i2c)

    def get_time(self) -> int:
        return timegm(self.ds3231.datetime)

    def set_time(self, epoch_time: int):
        self.ds3231.datetime = gmtime(epoch_time)

    def get_temp(self):
        return self.ds3231.temperature

    def increment_rtc(self, dt: int):
        t0 = self.get_time()
        self.set_time(t0 + dt)

    def set_system_time_from_rtc(self):
        self.set_system_time(self.get_time())

    @staticmethod
    def set_system_time(time):
        clock_settime(0, float(time))
