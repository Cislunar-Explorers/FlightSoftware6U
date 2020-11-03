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

# another thing to test:
# bash rc file
# append it
# having bootup run immediately
# ~/.bashrc
# idk ask toby


class BootUpMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)


        logger.info("Boot up beginning...")
        logger.info("Time when sleep starts: " + str(datetime.now()))
        time.sleep(BOOTUP_SEPARATION_DELAY)
        logger.info("Time when sleep stops: " + str(datetime.now()))

        logger.info("Creating DB session...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        # create the directory on the pi, this way next time a Restart is called
        os.mkdir(LOG_DIR)

        logger.info("Logging info to DB...")
        self.log()
        self.selected = None

        # deploy antennae
        #logger.info("Beginning burn wire...")
        #parent.gom.burnwire1(5)

    def run_mode(self):
        # initialize the cameras, select a camera
        # TODO is this done right?
        logger.info("Creating camera mux...")
        mux = camera.CameraMux()
        logger.info("Is camera detected? " + str(mux.detect()))
        if mux.detect():
            logger.info("Cam detected")
            self.selected = True
            mux.selectCamera(random.choice(1, 3, 4))
        else:
            logger.info("Cam not detected...")
            self.selected = False
            logger.info("Restarting...")

            # TODO make it so that main runs
            # this will restart the Pi
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

    def init_spin(self):
        pass


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
        cam = random.choice(1, 3, 4)
        mux = camera.CameraMux()
        mux.selectCamera(cam)
        camera.Camera.initialize()
        logger.info("Camera detected? " + str(mux.detect()))
