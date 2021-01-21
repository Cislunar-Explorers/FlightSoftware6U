import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path, RebootsModel
from utils.constants import DB_FILE, BOOTUP_SEPARATION_DELAY
import OpticalNavigation.core.camera as camera
from flight_modes.flight_mode import FlightMode
import os
import random
from utils.log import get_log
from utils.constants import FMEnum, BootCommandEnum, RestartCommandEnum

logger = get_log()


class BootUpMode(FlightMode):
    flight_mode_id = FMEnum.Boot.value
    command_codecs = {BootCommandEnum.Split.value: ([], 0)}
    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)

        logger.debug("Boot up beginning...")
        logger.debug("Time when sleep starts: " + str(datetime.now()))
        time.sleep(BOOTUP_SEPARATION_DELAY)
        logger.debug("Time when sleep stops: " + str(datetime.now()))

        logger.debug("Creating DB session...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        logger.debug("Logging info to DB...")
        self.log()

        # deploy antennae
        logger.info("Beginning burn wire...")
        parent.gom.burnwire1(5)

    def run_mode(self):
        logger.debug("run_mode running (nothing happens)")
        pass

        logger.debug("Transferring to RestartMode via sudo reboot")
        os.system("sudo reboot")

    def log(self):
        is_bootup = True
        reboot_at = psutil.boot_time()
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()
        logger.debug("Log to DB complete...")

    def update_state(self) -> int:
        logger.debug("updating state... doesnt do nothin")
        # os.system("sudo reboot")
        #return 1  # restart mode


class RestartMode(FlightMode):
    command_codecs = {}
    command_arg_unpackers = {}
    flight_mode_id = FMEnum.Restart.value

    def __init__(self, parent):
        super().__init__(parent)

        logger.debug("Restarting...")
        logger.debug("Creating DB session...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        # add info about restart to database
        logger.debug("Logging to DB...")
        self.log()

    def log(self):
        is_bootup = False
        reboot_at = psutil.boot_time()
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()
        logger.debug("Logging to DB complete...")

    # TODO implement error handling for if camera not detected
    def run_mode(self):
        logger.debug("run_mode running (nothing happens)")
        pass

        # logger.debug("Taking raw observation to test")
        # cam_object.rawObservation("restart_cam_test.mjpeg")

        """how to see the DB...
        boots = self.session.query(RebootsModel).all()
        for boot in boots:
            print(boot)"""

    def update_state(self) -> int:
        logger.debug("updating state... will now transfer to normal")
        return 2  # restart mode

