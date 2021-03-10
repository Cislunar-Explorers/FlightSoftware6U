import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path, RebootsModel
from utils.constants import DB_FILE, BOOTUP_SEPARATION_DELAY
from flight_modes.flight_mode import FlightMode
import os
from utils.log import get_log
from utils.constants import FMEnum, BootCommandEnum, RestartCommandEnum
import psutil

logger = get_log()


class BootUpMode(FlightMode):
    flight_mode_id = FMEnum.Boot.value
    command_codecs = {BootCommandEnum.Split.value: ([], 0)}
    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        logger.info("Boot up beginning...")
        time.sleep(BOOTUP_SEPARATION_DELAY)

        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        self.log()

        # deploy antennae
        logger.info("Antennae deploy...")
        parent.gom.burnwire1(5)

        os.system("sudo reboot")

    def log(self):
        is_bootup = True
        reboot_at = datetime.fromtimestamp(psutil.boot_time())
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()

    def update_state(self) -> int:
        return 0


class RestartMode(FlightMode):
    command_codecs = {}
    command_arg_unpackers = {}
    flight_mode_id = FMEnum.Restart.value

    def __init__(self, parent):
        super().__init__(parent)

        logger.info("Restarting...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        # add info about restart to database
        self.log()

    def log(self):
        is_bootup = False
        reboot_at = datetime.fromtimestamp(psutil.boot_time())
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()

    # TODO implement error handling for if camera not detected
    def run_mode(self):
        pass

        # logger.debug("Taking raw observation to test")
        # cam_object.rawObservation("restart_cam_test.mjpeg")

        """how to see the DB...
        boots = self.session.query(RebootsModel).all()
        for boot in boots:
            print(boot)"""

    def update_state(self) -> int:
        return 2