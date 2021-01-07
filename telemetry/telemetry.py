from main import MainSatelliteThread
import time
import psutil
from uptime import uptime
from drivers.sensor import SynchronousSensor


class GomSensor(SynchronousSensor):
    def __init__(self, parent: MainSatelliteThread):
        super().__init__(parent)
        self.hk = None  # HouseKeeping data: eps_hk_t struct
        self.hkparam = None  # hkparam_t struct
        self.percent = float()

    def poll(self):
        super().poll()
        self.hk = self.parent.gom.get_health_data(level="eps")
        self.hkparam = self.parent.gom.get_health_data()
        self.percent = self.parent.gom.read_battery_percentage()


class GyroSensor(SynchronousSensor):
    def __init__(self, parent: MainSatelliteThread):
        super().__init__(parent)
        self.xrot = float()  # rad/s
        self.yrot = float()
        self.zrot = float()

        self.xmag = float()  # nanoTesla
        self.ymag = float()
        self.zmag = float()

        self.xacc = float()  # m/s^2
        self.yacc = float()
        self.zacc = float()

    def poll(self):
        super().poll()
        self.xrot, self.yrot, self.zrot = self.parent.gyro.gyroscope
        raise NotImplementedError("Need to add IMU and MAG measurements and data smoothing")


class PressureSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.pressure = float()  # pressure, psi

    def poll(self):
        super().poll()
        self.pressure = self.parent.adc


class ThermocoupleSensor(SynchronousSensor):
    def __init__(self, parent):
        super().__init__(parent)
        self.temperature = float()  # Fuel tank temperature, deg C

    def poll(self):
        super().poll()
        self.temperature = self.parent.adc


class PiSensor(SynchronousSensor):
    def __init__(self, parent: MainSatelliteThread):
        super().__init__(parent)
        self.cpu = float()
        self.ram = float()
        self.disk = float()
        self.boot_time = float()
        self.up_time = float()

    def poll(self):
        super().poll()
        self.cpu = psutil.cpu_percent()
        self.ram = psutil.virtual_memory().percent
        self.disk = psutil.disk_usage("/").percent
        self.boot_time = psutil.boot_time()
        self.up_time = uptime()


class Telemetry:
    def __init__(self, parent: MainSatelliteThread):
        # The purpose of the parent object is to ensure that only one object is defined for each sensor/component
        # So almost all calls to sensors will be made through self.parent.<insert sensor stuff here>
        self.parent = parent

        self.gom = GomSensor(parent)
        self.gyr = GyroSensor(parent)
        self.prs = PressureSensor(parent)
        self.thm = ThermocoupleSensor(parent)
        self.rpi = PiSensor(parent)

        self.sensors = [self.gom, self.gyr, self.prs, self.thm, self.rpi]

        # initialize databases here if not init'd already

    def poll_telem(self):
        # polls every sensor for the latest telemetry that can be accessed
        # by the rest of the software.
        for sensor in self.sensors:
            sensor.poll()


    def poll_gyro(self):
        rot_data = []
        n = 50
        d = 1
        for i in range(n):
            rot_data.append(self.parent.gyro.gyroscope)
            time.sleep(1.0 / n)
        # Take ~50 measurements over 1 sec and take the average
        pass


    def write_telem(self, telem):
        # writes telem to database, where telem is either only one of the outputs of one of the poll_<sensor>
        # functions above, or a list of all of them.
        pass

    def query_telem(self):
        # querys telemetry from database
        pass
