import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path, RebootsModel
from utils.constants import DB_FILE, BOOTUP_SEPARATION_DELAY, NO_FM_CHANGE
from flight_modes.flight_mode import FlightMode
import os
import logging
from utils.constants import FMEnum
import psutil


class BootUpMode(FlightMode):
    """FMID 0"""

    flight_mode_id = FMEnum.Boot.value

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        logging.info("Boot up beginning...")
        time.sleep(BOOTUP_SEPARATION_DELAY)

        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        self.log()

        # deploy antennae
        # FIXME: differentiate between Hydrogen and Oxygen. Each satellite now has different required Bootup behaviors
        logging.info("Antennae deploy...")
        self._parent.gom.burnwire1(5)

        if self._parent.need_to_reboot:
            # TODO: double check the boot db history to make sure we aren't going into a boot loop
            # TODO: downlink something to let ground station know we're alive
            logging.critical("Rebooting to init cameras")
            os.system("sudo reboot")

    def log(self):
        is_bootup = True
        reboot_at = datetime.fromtimestamp(psutil.boot_time())
        new_bootup = RebootsModel(is_bootup=is_bootup, reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()

    def update_state(self) -> int:
        return NO_FM_CHANGE


class RestartMode(FlightMode):
    """FMID 1"""

    flight_mode_id = FMEnum.Restart.value

    def __init__(self, parent):
        super().__init__(parent)

        logging.info("Restarting...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        self.log()

    def log(self):
        is_bootup = False
        reboot_at = datetime.fromtimestamp(psutil.boot_time())
        new_bootup = RebootsModel(is_bootup=is_bootup, reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()

    # TODO implement error handling for if camera not detected
    def run_mode(self):
        if self._parent.need_to_reboot:
            # TODO double check the boot db history to make sure we aren't going into a boot loop
            # TODO: downlink something to let ground station know we're alive and going to reboot
            logging.critical("Rebooting to init cameras")
            os.system("sudo reboot")

        self.completed_task()

    def update_state(self) -> int:
        return super().update_state()
