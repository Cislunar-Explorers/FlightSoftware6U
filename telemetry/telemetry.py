from main import MainSatelliteThread


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

        gom_houskeeping_struct_1 = self.parent.gom.get_health_data(level="default")
        gom_houskeeping_struct_2 = self.parent.gom.get_health_data(level="eps")
        ...

    def poll_gyro(self):
        # poll current angular velocity and/or acceleration
        pass

    def poll_rpi(self):
        # gets pi's temperature, CPU/Ram/disk utilization, and other useful data
        pass

    def poll_pressure(self):
        # get's pressure in the fuel chamber
        pass

    def write_telem(self, telem):
        # writes telem to database, where telem is either only one of the outputs of one of the poll_<sensor>
        # functions above, or a list of all of them.
        pass

    def query_telem(self):
        # querys telemetry from database
        pass
