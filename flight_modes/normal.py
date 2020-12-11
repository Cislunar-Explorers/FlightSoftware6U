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

    flight_mode_id = FMEnum.Normal.value

    command_codecs = {
        NormalCommandEnum.RunOpNav.value: ([], 0),
        NormalCommandEnum.SetDesiredAttitude.value: (
            [ATTITUDE_X, ATTITUDE_Y, ATTITUDE_Z],
            24,
        ),
        NormalCommandEnum.SetAccelerate.value: ([ACCELERATE], 1),
        # NormalCommandEnum.SetBreakpoint.value: ([], 0),  # TODO define exact parameters
    }

    command_arg_unpackers = {
        ATTITUDE_X: (pack_double, unpack_double),
        ATTITUDE_Y: (pack_double, unpack_double),
        ATTITUDE_Z: (pack_double, unpack_double),
        ACCELERATE: (pack_bool, unpack_bool),
    }

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        print("Execute normal mode")