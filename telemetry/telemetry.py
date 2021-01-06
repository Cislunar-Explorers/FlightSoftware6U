from main import MainSatelliteThread
import time
import psutil
from uptime import uptime


class Telemetry:
    def __init__(self, parent: MainSatelliteThread):
        # The purpose of the parent object is to ensure that only one object is defined for each sensor/component
        # So almost all calls to sensors will be made through self.parent.<insert sensor stuff here>
        self.parent = parent

        # self.latest stores the latest telemetry gathered. So another piece of software can get the most recent
        # known telem
        self.latest = {"gom": None,
                       "gyro": None,
                       "pressure": None,
                       "rpi": None}

        # initialize databases here if not init'd already

    def poll_telem(self):
        # polls every sensor for the latest telemetry and stores a state variable (self.latest) that can be accessed
        # by the rest of the software.
        pass

    def poll_gom(self):
        # polls gom telemetry

        hkparam = self.parent.gom.get_health_data()
        hkparam_time = time.time()
        eps_hk = self.parent.gom.get_health_data(level="eps")
        eps_hk_time = time.time()

        return (hkparam, hkparam_time), (eps_hk, eps_hk_time)

    def poll_gyro(self):
        rot_data = []
        n = 50
        d = 1
        for i in range(n):
            rot_data.append(self.parent.gyro.gyroscope)
            time.sleep(1.0 / n)
        # Take ~50 measurements over 1 sec and take the average
        pass

    def poll_rpi(self):
        # gets pi's temperature, CPU/Ram/disk utilization, and other useful data
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        disk = psutil.disk_usage("/").percent
        boot_time = psutil.boot_time()
        up_time = uptime()
        # temperature =
        pass

    def poll_pressure(self):
        # gets pressure in the fuel chamber
        pressure = self.parent.adc
        pass

    def write_telem(self, telem):
        # writes telem to database, where telem is either only one of the outputs of one of the poll_<sensor>
        # functions above, or a list of all of them.
        pass

    def query_telem(self):
        # querys telemetry from database
        pass
