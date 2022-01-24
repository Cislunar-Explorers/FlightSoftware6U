import logging
from time import sleep, time

from flight_modes.flight_mode import FlightMode
from utils import parameters as params
from utils.constants import *


class LowBatterySafetyMode(FlightMode):
    """FMID 3: Low Battery Safety Mode: the main goal here is to save power as much as possible in case we are in a
    low-power state. We enter Low battery mode whenever we are low on battery (duh...) as defined by
    parameters.ENTER_LOW_BATTERY_MODE_THRESHOLD and whenever we aren't getting any current from our solar panels,
    which can happen in either an eclipse, or if our harnessing or solar panels break.

    In Low Battery Mode we want to downlink only the most critical data for keeping the spacecraft alive. Downlinking
    data requires a huge amount of electrical power (to power our RF Power Amplifier) so we want to minimize the
    frequency and the size of downlinks in this mode """

    flight_mode_id = FMEnum.LowBatterySafety.value

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        if self.task_completed:
            sleep(params.LOW_BATT_MODE_SLEEP)  # saves battery, maybe?
        else:
            if self._parent.gom is not None:
                # turn off all devices except for LNA. Most of this stuff is redundant, but better safe than sorry
                self._parent.gom.power_amplifier.set(False)  # turn off PA loadswitch
                self._parent.gom.rf_tx.set(False)  # Switch RF switch to RX
                self._parent.gom.rf_rx.set(True)  # Switch RF switch to RX

                self._parent.gom.electrolyzers.set(False)  # stop electrolysis
                self._parent.gom.burnwire.set(False)
                self._parent.gom.glowplug_1.set(False)
                self._parent.gom.glowplug_2.set(False)
                self._parent.gom.solenoid.set(False)

            self.completed_task()

    def poll_inputs(self):
        super().poll_inputs()

    def update_state(self) -> int:
        # if there are maneuvers, reorientations, opnav, to be done, then switch
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm

        # check power supply to see if I can transition back to NormalMode
        if self._parent.telemetry.gom.hk.vbatt > params.EXIT_LOW_BATTERY_MODE_THRESHOLD:
            return FMEnum.Normal.value

        time_for_opnav = (
            time() - self._parent.last_opnav_run
        ) // 60 < params.LB_OPNAV_INTERVAL

        time_for_telem = (
            time() - self._parent.radio.last_transmit_time
        ) // 60 < params.LB_TLM_INTERVAL

        if time_for_opnav:  # TODO: and FMEnum.OpNav.value not in self._parent.FMQueue:
            self._parent.FMQueue.put(FMEnum.OpNav.value)

        if time_for_telem:
            # Add a minimal data packet to our periodic downlink beacon
            telem = self._parent.telemetry.minimal_packet()
            downlink = self._parent.command_handler.pack_telemetry(
                CommandEnum.CritTelem, **telem
            )
            self._parent.downlink_queue.put(downlink)
            logging.info("Added a minimal data packet to our downlink")

        return NO_FM_CHANGE
