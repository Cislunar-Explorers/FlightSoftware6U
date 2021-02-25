import board
import busio
import adafruit_ds3231
from time import gmtime
from calendar import timegm


class RTC:
    """Interface class for the Adafruit DS3231. This clock only supports second-level accuracy, but is extremely
    precise and will run whenever the gom has any power."""

    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ds3231 = adafruit_ds3231.DS3231(i2c)

    def get_time(self):
        return timegm(self.ds3231.datetime)

    def set_time(self, epoch_time):
        self.ds3231.datetime(gmtime(epoch_time))
