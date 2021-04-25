import time
from datetime import datetime
from utils.db import create_sensor_tables_from_path, RebootsModel
from utils.constants import DB_FILE, BOOTUP_SEPARATION_DELAY, NO_FM_CHANGE, FMEnum
from flight_modes.flight_mode import FlightMode
import os
from utils.log import get_log
from utils.constants import FMEnum, BootCommandEnum, RestartCommandEnum, MissionModeEnum
import psutil
import utils.parameters as params
import glob

logger = get_log()


class BootUpMode(FlightMode):
    """FMID 0"""
    flight_mode_id = FMEnum.Boot.value
    command_codecs = {
        BootCommandEnum.Switch.value: ([], 0),
        BootCommandEnum.Split.value: ([], 0)
    }

    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        logger.info("Boot up beginning...")
        time.sleep(BOOTUP_SEPARATION_DELAY)

        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

        self.log()
        self.set_parameters()

        if params.SPACECRAFT_NAME == 'HYDROGEN':
            # deploy antennae
            logger.info("Deploying antennae on Hydrogen")
            self.parent.gom.burnwire1(params.SPLIT_BURNWIRE_DURATION)
            self.parent.command_definitions.set_parameter(name="BURNWIRES_FIRED", value=True, hard_set=True)

        if self.parent.need_to_reboot:
            # TODO: double check the boot db history to make sure we aren't going into a boot loop
            # TODO: downlink something to let ground station know we're alive
            logger.critical("Rebooting to init cameras")
            os.system("sudo reboot")

    def log(self):
        is_bootup = True
        reboot_at = datetime.fromtimestamp(psutil.boot_time())
        new_bootup = RebootsModel(is_bootup=is_bootup,
                                  reboot_at=reboot_at)
        self.session.add(new_bootup)
        self.session.commit()

    def update_state(self) -> int:
        return NO_FM_CHANGE

    def set_parameters(self):
        self.parent.command_definitions.set_parameter(name="SPACECRAFT_NAME", value=determine_name(), hard_set=True)
        self.parent.logger.info(f"Spacecraft is {params.SPACECRAFT_NAME}")
        self.update_mission_mode(MissionModeEnum.Boot.value)


class RestartMode(FlightMode):
    """FMID 1"""
    command_codecs = {RestartCommandEnum.Switch.value: ([], 0)}
    command_arg_unpackers = {}
    flight_mode_id = FMEnum.Restart.value

    def __init__(self, parent):
        super().__init__(parent)

        logger.info("Restarting...")
        create_session = create_sensor_tables_from_path(DB_FILE)
        self.session = create_session()

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
        if self.parent.need_to_reboot:
            # TODO double check the boot db history to make sure we aren't going into a boot loop
            # TODO: downlink something to let ground station know we're alive and going to reboot
            logger.critical("Rebooting to init cameras")
            os.system("sudo reboot")

        self.completed_task()

    def update_state(self) -> int:
        return super().update_state()


def get_hostname():
    # thank you https://stackoverflow.com/questions/4271740/how-can-i-use-python-to-get-the-system-hostname
    # Does not work on windows
    return os.uname()[1]


def determine_name():
    """Checks if Hydrogen or oxygen"""
    filenames = glob.glob('../utils/*_parameters.py')
    if 'hydrogen' in ','.join(filenames):
        space_name = 'HYDROGEN'
    else:
        space_name = 'OXYGEN'

    # if in flight, double check the hostname
    # Want to assume oxygen in worst case scenario (don't want to split prematurely!)
    if params.FOR_FLIGHT:
        hostname = get_hostname()
        if 'oxygen' in hostname and space_name == 'HYDROGEN':
            space_name = 'OXYGEN'

    return space_name
