import time
import psutil
from uptime import uptime
from drivers.sensor import SynchronousSensor, SensorError
import numpy as np


def moving_average(x, w):
    """x: 1-D data, w: number of samples to average over."""
    return np.convolve(x, np.ones(w), 'valid') / w


class GomSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.hk = None  # HouseKeeping data: eps_hk_t struct
        self.hkparam = None  # hkparam_t struct
        self.percent = float()

    def poll(self):
        super().poll()
        self.hk = self.parent.gom.get_health_data(level="eps")
        self.hkparam = self.parent.gom.get_health_data()
        self.percent = self.parent.gom.read_battery_percentage()


class GomSensorError(SensorError):
    pass


class GyroSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.rot = (float(), float(), float())  # rad/s
        self.mag = (float(), float(), float())  # microTesla
        self.acc = (float(), float(), float())  # m/s^2

    def poll(self):
        super().poll()
        self.rot = self.parent.gyro.get_gyro()
        self.mag = self.parent.gyro.get_mag()
        self.acc = self.parent.gyro.get_acceleration()

    def poll_smoothed(self, freq=10, duration=3, samples=5):
        # poll and smooth gyro data

        n_data = freq * duration
        data = [] * n_data
        for i in range(n_data):
            data[i] = self.parent.gyro.get_gyro()
            time.sleep(1.0 / freq)

        data = np.asarray(data)

        smoothed = np.empty(3)

        # smooth data using convolution
        smoothed[0] = moving_average(data.T[0], samples)
        smoothed[1] = moving_average(data.T[1], samples)
        smoothed[2] = moving_average(data.T[2], samples)

        return smoothed


class GyroError(SensorError):
    pass


class PressureSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.pressure = float()  # pressure, psi

    def poll(self):
        super().poll()
        self.pressure = self.parent.adc.read_pressure()


class PressureError(SensorError):
    pass


class ThermocoupleSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.temperature = float()  # Fuel tank temperature, deg C

    def poll(self):
        super().poll()
        self.temperature = self.parent.adc.read_temperature()


class ThermocoupleError(SensorError):
    pass


class PiSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.cpu = int()
        self.ram = int()
        self.disk = int()
        self.boot_time = float()
        self.up_time = int()

    def poll(self):
        super().poll()
        self.cpu = int(psutil.cpu_percent())
        self.ram = int(psutil.virtual_memory().percent)
        self.disk = int(psutil.disk_usage("/").percent)
        self.boot_time = psutil.boot_time()
        self.up_time = int(uptime())

    def all(self):
        return (self.cpu,
                self.ram,
                self.disk,
                self.boot_time,
                self.up_time)


class PiSensorError(SensorError):
    pass


class Telemetry(SynchronousSensor):
    def __init__(self, parent):
        # The purpose of the parent object is to ensure that only one object is defined for each sensor/component
        # So almost all calls to sensors will be made through self.parent.<insert sensor stuff here>
        super().__init__(parent)

        self.gom = GomSensor(parent)
        self.gyr = GyroSensor(parent)
        self.prs = PressureSensor(parent)
        self.thm = ThermocoupleSensor(parent)
        self.rpi = PiSensor(parent)

        self.sensors = [self.gom, self.gyr, self.prs, self.thm, self.rpi]

        # initialize databases here if not init'd already

    def poll(self):
        # polls every sensor for the latest telemetry that can be accessed
        # by the rest of the software.
        super().poll()
        for sensor in self.sensors:
            sensor.poll()

    def sensor_check(self):
        """Makes sure that the values returned by the sensors are reasonable"""

        # if data is over an hour old, poll new data
        if (time.time() - self.poll_time) > 3600:
            self.poll()

        if any(i > self.parent.constants.MAX_GYRO_RATE for i in tuple(map(abs, self.gyr.rot))):
            self.parent.logger.error("Gyro not functioning properly")
            raise GyroError(f"Unreasonable gyro values: {self.gyr.rot}")

        if self.prs.pressure < 0 or self.prs.pressure > 2000:
            self.parent.logger.error("Pressure sensor not functioning properly")
            raise PressureError(f"Unreasonable pressure: {self.prs.pressure}")

        if self.thm.temperature < -200 or self.thm.temperature > 200:
            self.parent.logger.error("Thermocouple not functioning properly")
            raise ThermocoupleError(f"Unreasonable fuel tank temperature: {self.thm.temperature}")

        if self.gom.percent < 0 or self.gom.percent > 1.5:
            self.parent.logger.error("Gom HK not functioning properly")
            raise GomSensorError(f"Unreasonable battery percentage: {self.gom.percent}")

        if any(i < 0 for i in self.rpi.all()):
            self.parent.logger.error("RPi sensors not functioning properly")
            raise PiSensorError

    def write_telem(self, telem):
        # writes telem to database, where telem is either only one of the outputs of one of the poll_<sensor>
        # functions above, or a list of all of them.
        raise NotImplementedError

    def query_telem(self):
        # querys telemetry from database
        raise NotImplementedError
