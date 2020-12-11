import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path, RebootsModel
from utils.constants import DB_FILE, BOOTUP_SEPARATION_DELAY
import OpticalNavigation.core.camera as camera
from flight_modes.flight_mode import FlightMode
import os
import random
from utils.log import get_log

logger = get_log()

class NormalMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)
        logger.info("Now in normal mode...")
        self.run_mode()

    def run_mode(self):
        logger.info("Checking sensors...")
        self.check_sensors()
        logger.info("Checking battery...")
        self.check_battery()
        logger.info("Now running op nav...")
        self.run_opnav()
        logger.info("Now downlinking data...")
        self.downlink()
        logger.info("Waiting for next interval to run normal mode")

    def check_sensors(self):
        pass

    def check_battery(self):
        pass

    def run_opnav(self):
        pass

    def downlink(self):
        pass