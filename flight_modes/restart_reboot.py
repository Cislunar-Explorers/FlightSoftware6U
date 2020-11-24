import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path, RebootsModel
from utils.constants import DB_FILE, BOOTUP_SEPARATION_DELAY, LOG_DIR
import OpticalNavigation.core.camera as camera
from flight_modes.flight_mode import FlightMode
import os
import random
from utils.log import get_log

logger = get_log()

class BootUpMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

        logger.info("Boot up beginning...")
        logger.info("Time when sleep starts: " + str(datetime.now()))
        #time.sleep(BOOTUP_SEPARATION_DELAY)
        time.sleep(5)
        logger.info("Time when sleep stops: " + str(datetime.now()))

        logger.info("Creating DB session...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        logger.info("Logging info to DB...")
        self.log()

        # deploy antennae
        #logger.info("Beginning burn wire...")
        #parent.gom.burnwire1(5)

    def run_mode(self):
        # initialize the cameras, select a camera
        # TODO is this done right?
        logger.info("Creating camera mux...")
        mux = camera.CameraMux()
        logger.info("Selecting a camera...")
        mux.selectCamera(random.choice([1, 2, 3]))
        logger.info("Transferring to RestartMode")
        os.system("sudo reboot")

    # will add info about the boot up to the db
    def log(self):
        is_bootup = True
        reboot_at = datetime.now()
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()
        logger.info("Log to DB complete...")


class RestartMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

        logger.info("Restarting...")
        logger.info("Creating DB session...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        # add info about restart to database
        logger.info("Logging to DB...")
        self.log()

    # this should add to the RebootsModel
    # will add info about the bootup to the db
    def log(self):
        is_bootup = False
        reboot_at = datetime.now()
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()
        logger.info("Logging to DB complete...")

    def run_mode(self):
        # initialize the cameras, select a random camera
        logger.info("Selecting a camera")
        mux = camera.CameraMux()
        mux.selectCamera(random.choice([1, 2, 3]))
        cam_object = camera.Camera()
        # cam_object.initialize()
        logger.info("Taking raw observation to test")
        cam_object.rawObservation("restart_cam_test.mjpeg")

        logger.info("Displaying sqlalchemy model:")
        boots = self.session.query(RebootsModel).all()
        for boot in boots:
            print(boot)

