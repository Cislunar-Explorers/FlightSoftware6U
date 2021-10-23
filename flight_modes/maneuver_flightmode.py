from time import sleep, time

from .flight_mode import PauseBackgroundMode
from drivers.ADCDriver import ADC
from utils.constants import FMEnum, NO_FM_CHANGE, GLOWPLUG_DURATION, NormalCommandEnum
from utils.log import get_log
from utils.parameter_utils import set_parameter
import utils.constants as consts
import utils.parameters as params

logger = get_log()

GLOW_WAIT_TIME = 2
PRESSURE_THRESHOLD = 1000  # TODO
PRESSURE_DELTA = 10


class ManeuverMode(PauseBackgroundMode):
    flight_mode_id = FMEnum.Maneuver.value
    command_codecs = {}
    command_arg_unpackers = {}
    glowplugs = [
        {"name": "glowplug", "valid": True},
        {"name": "glowplug2", "valid": True}
    ]
    glowplug_index = 0

    def __init__(self, parent):
        super().__init__(parent)

    # TODO use other definition
    def set_parameter(self, **kwargs):
        """Changes the values of a parameter in utils/parameters.py or .json if hard_set"""
        name = kwargs[consts.NAME]
        value = kwargs[consts.VALUE]
        hard_set = kwargs[consts.HARD_SET]
        initial_value = set_parameter(name, value, hard_set)

        acknowledgement = self._parent.downlink_handler.pack_downlink(
            self._parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.SetParam.value, successful=True)
        self._parent.downlink_queue.put(acknowledgement)

        self._parent.logger.info(f"Changed constant {name} from {initial_value} to {value}")

    def valid_glowplug(self, current_pressure, prior_pressure):
        """Check pressure in place of acceleration for glowplug validation."""
        return current_pressure >= PRESSURE_THRESHOLD and current_pressure - prior_pressure >= PRESSURE_DELTA

    def update_state(self) -> int:
        if self.task_completed is True:
            logger.info("Maneuver complete. Exiting maneuver mode...")
            return FMEnum.Normal.value
        return NO_FM_CHANGE

    def run_mode(self):
        """Activate glowplugs until one works."""
        if params.SCHEDULED_BURN_TIME < time():
            # TODO send info to ground??
            logger.info("Maneuver time passed. Skipped.")
        else:
            # TODO what if SCHEDULED_BURN_TIME is too far into the future
            # sleeping for 5 fewer seconds than the delay for safety
            sleep(min(0, (params.SCHEDULED_BURN_TIME - time()) - 5))
            logger.info("Glowplug maneuver...")
            self.task_completed = False

            while not self.task_completed:
                prior_pressure = self._parent.adc.read_pressure()
                glowplug_function = self.glowplugs[self.glowplug_index]["name"]
                getattr(self._parent.gom, glowplug_function)(GLOWPLUG_DURATION)
                sleep(GLOW_WAIT_TIME)
                current_pressure = self._parent.adc.read_pressure()
                if self.valid_glowplug(current_pressure, prior_pressure):
                    self.task_completed = True
                else:
                    if self.glowplug_index == len(self.glowplugs) - 1:
                        # TODO log?? failure of all glow plugs... do a final check?
                        return
                    self.glow_plug_index += 1

        # prepare next maneuver
        smallest_time_burn = None
        while not self._parent.maneuver_queue.empty():
            smallest_time_burn = self._parent.maneuver_queue.get()
            if smallest_time_burn < time():
                # TODO send info to ground??
                smallest_time_burn = None
                logger.info("Maneuver time passed. Skipped.")
            else:
                break

        self.set_parameter(name="SCHEDULED_BURN_TIME", value=smallest_time_burn, hard_set=True)
