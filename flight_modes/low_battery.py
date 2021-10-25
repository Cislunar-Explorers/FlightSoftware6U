from time import sleep, time

from flight_modes.flight_mode import FlightMode
from utils import parameters as params
from utils.constants import *
import logging


class LowBatterySafetyMode(FlightMode):
    """FMID 3: Low Battery Safety Mode: the main goal here is to save power as much as possible in case we are in a
    low-power state. We enter Low battery mode whenever we are low on battery (duh...) as defined by
    parameters.ENTER_LOW_BATTERY_MODE_THRESHOLD and whenever we aren't getting any current from our solar panels,
    which can happen in either an eclipse, or if our harnessing or solar panels break.

    In Low Battery Mode we want to downlink only the most critical data for keeping the spacecraft alive. Downlinking
    data requires a huge amount of electrical power (to power our RF Power Amplifier) so we want to minimize the
    frequency and the size of downlinks in this mode """

    flight_mode_id = FMEnum.LowBatterySafety.value

    command_codecs = {}
    command_arg_types = {}
    downlink_codecs = {
        LowBatterySafetyCommandEnum.BasicTelem.value: ([RTC_TIME, ATT_1, ATT_2, ATT_3, ATT_4,
                                                        HK_TEMP_1, HK_TEMP_2, HK_TEMP_3, HK_TEMP_4, GYRO_TEMP,
                                                        THERMOCOUPLER_TEMP,
                                                        CURRENT_IN_1, CURRENT_IN_2, CURRENT_IN_3,
                                                        VBOOST_1, VBOOST_2, VBOOST_3, SYSTEM_CURRENT, BATTERY_VOLTAGE,
                                                        PROP_TANK_PRESSURE], 84),

        LowBatterySafetyCommandEnum.CritTelem.value: (
            [BATTERY_VOLTAGE, SUN_CURRENT, SYSTEM_CURRENT, BATT_MODE, PPT_MODE], 8)
    }

    downlink_arg_types = {RTC_TIME: 'double',
                          POSITION_X: 'double', POSITION_Y: 'double', POSITION_Z: 'double',
                          ATT_1: 'float', ATT_2: 'float', ATT_3: 'float', ATT_4: 'float',
                          HK_TEMP_1: 'short', HK_TEMP_2: 'short', HK_TEMP_3: 'short', HK_TEMP_4: 'short',
                          GYRO_TEMP: 'float',
                          THERMOCOUPLER_TEMP: 'float',
                          CURRENT_IN_1: 'short', CURRENT_IN_2: 'short', CURRENT_IN_3: 'short',
                          VBOOST_1: 'short', VBOOST_2: 'short', VBOOST_3: 'short',
                          PROP_TANK_PRESSURE: 'float',
                          SUCCESSFUL: 'bool',

                          BATTERY_VOLTAGE: 'short',
                          SUN_CURRENT: 'short',
                          SYSTEM_CURRENT: 'short',
                          BATT_MODE: 'uint8',
                          PPT_MODE: 'uint8',
                          }

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        if self.task_completed:
            sleep(params.LOW_BATT_MODE_SLEEP)  # saves battery, maybe?
        else:
            if self._parent.gom is not None:
                # turn off all devices except for LNA. Most of this stuff is redundant, but better safe than sorry
                self._parent.gom.set_pa(False)
                self._parent.gom.rf_receiving_switch(receive=True)
                self._parent.gom.rf_transmitting_switch(receive=True)
                self._parent.gom.set_electrolysis(False)
                self._parent.gom.burnwire1(False)
                self._parent.gom.glowplug(False)
                self._parent.gom.glowplug2(False)
                self._parent.gom.pc.set_single_output(4, False, 0)

            self.completed_task()

    def poll_inputs(self):
        super().poll_inputs()

    def update_state(self) -> int:
        super_fm = super().update_state()  # if there are maneuvers, reorientations, opnav, to be done, then switch
        if super_fm != NO_FM_CHANGE:
            return super_fm

        # check power supply to see if I can transition back to NormalMode
        if self._parent.telemetry.gom.hk.vbatt > params.EXIT_LOW_BATTERY_MODE_THRESHOLD:
            return FMEnum.Normal.value

        time_for_opnav = (time() - self._parent.last_opnav_run) // 60 < params.LB_OPNAV_INTERVAL
        time_for_telem = (time() - self._parent.radio.last_transmit_time) // 60 < params.LB_TLM_INTERVAL

        if time_for_opnav:  # TODO: and FMEnum.OpNav.value not in self._parent.FMQueue:
            self._parent.FMQueue.put(FMEnum.OpNav.value)

        if time_for_telem:
            # Add a minimal data packet to our periodic downlink beacon
            telem = self._parent.telemetry.minimal_packet()
            downlink = self._parent.downlink_handler.pack_downlink(self._parent.downlink_counter,
                                                                   FMEnum.LowBatterySafety.value,
                                                                   LowBatterySafetyCommandEnum.CritTelem.value,
                                                                   **telem)
            self._parent.downlink_queue.put(downlink)
            logging.info("Added a minimal data packet to our downlink")

        return NO_FM_CHANGE
