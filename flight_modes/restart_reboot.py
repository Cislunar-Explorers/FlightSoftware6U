import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path
from utils.constants import DB_FILE
from utils.constants import BOOTUP_SEPARATION_DELAY
from utils.db import RebootsModel
import OpticalNavigation.core.camera as camera
from flight_modes.flight_mode import FlightMode


class BootUpMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

        time.sleep(BOOTUP_SEPARATION_DELAY)
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        # add info about boot up to database
        self.log()

        # do the run_mode function inside flight_mode
        # no clue what this is really doing
        super().run_mode(self)

        # initialize the camera
        self.init_camera()

    # initialize the cameras, select a camera
    def init_camera(self):
        # TODO is this done right?
        mux = camera.CameraMux()
        assert mux.detect()

    # will add info about the bootup to the db
    def log(self):
        is_bootup = True
        reboot_at = datetime.now()
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()


class RestartMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        # add info about restart to database
        self.log()

        # do the run_mode function inside flight_mode
        # no clue what this is really doing
        super().run_mode(self)

        # double check that the cameras are good
        self.init_camera()

    # this should add to the RebootsModel
    # will add info about the bootup to the db
    def log(self):
        is_bootup = False
        reboot_at = datetime.now()
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()

    # initialize the cameras, select a camera
    def init_camera(self):
        cam = camera.Camera()
        # TODO now what?



