from drivers.device import Device
from adafruit_blinka.agnostic import board_id

if board_id and board_id != "GENERIC_LINUX_PC":
    import board
    import busio
    from time import gmtime, clock_settime
import adafruit_ds3231
from calendar import timegm


class RTC(Device):
    """Interface class for the Adafruit DS3231. This clock only supports second-level precision, but is extremely
    accurate and will run whenever the gom has any power."""

    driver: adafruit_ds3231.DS3231

    def __init__(self) -> None:
        super().__init__("RTC")

    def _connect_to_hardware(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.driver = adafruit_ds3231.DS3231(i2c)

    def _collect_telem(self):
        return self.get_time(), self.get_temp()

    def get_time(self) -> int:
        return timegm(self.driver.datetime)

    def set_time(self, epoch_time: int):
        self.driver.datetime = gmtime(epoch_time)

    def get_temp(self):
        return self.driver.temperature

    def increment_rtc(self, dt: int):
        t0 = self.get_time()
        self.set_time(t0 + dt)

    def set_system_time_from_rtc(self):
        self.set_system_time(self.get_time())

    @staticmethod
    def set_system_time(time):
        clock_settime(0, float(time))
