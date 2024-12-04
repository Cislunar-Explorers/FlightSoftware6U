import logging
from typing import Union
from fsw.drivers.power.power_controller import PA_EN, RF_RX_EN, RF_TX_EN, Power
from abc import ABC, abstractmethod
import time
from dataclasses import dataclass

from fsw.utils.constants import GomOutputs
import random


@dataclass
class LoadSwitchTelem:
    state: bool  # is the loadswitch on or off?


@dataclass
class P31uLoadSwitchTelem(LoadSwitchTelem):
    current_draw: int  # mA
    time_to_on: int  # seconds
    time_to_off: int  # seconds
    latchups: int  # number of latchups reported by the P31u


class LoadSwitch(ABC):
    """Abstract class for a loadswitch. Every device that can be turned on/off has
    an associated loadswitch which is represented by a concrete instantiation of this class"""

    def __init__(self, driver: Power) -> None:
        self.driver = driver
        self.telem: LoadSwitchTelem = self.get_new_telem()

    @abstractmethod
    def _set(self, state: bool):
        """Makes the hardware call to actuate the loadswitch"""
        ...

    def set(self, on: bool):
        """Sets the loadswitch to on (True) or off (False)"""
        self._set(on)
        self.get_new_telem()

    def get_telem(self) -> LoadSwitchTelem:
        """Returns the latest queued telemetry."""
        try:
            return self.telem
        except Exception:
            return self.get_new_telem()

    @abstractmethod
    def get_new_telem(self) -> LoadSwitchTelem:
        """Makes the hardware call to get new telemetry from the device"""
        ...

    def pulse(self, duration: Union[float, int], delay: Union[float, int] = 0):
        time.sleep(delay)
        self.set(True)
        time.sleep(duration)
        self.set(False)


class P31uLoadSwitch(LoadSwitch):
    """Loadswitch for devices connected to the Gomspace P31u"""

    p31u_output_id: GomOutputs

    def _set(self, state: bool, delay: int = 0):
        self.driver.set_single_output(self.p31u_output_id, int(state), delay)

    def set(self, on: bool, delay: int = 0):
        self._set(on, delay=delay)
        self.get_new_telem()

    def pulse(
        self,
        duration: Union[float, int],
        delay: Union[float, int] = 0,
        asynchronous: bool = False,
        on: bool = True,
    ):
        if asynchronous:
            # I think the gomspace p31u can only turn outputs on/off with second-level precision
            # so cast all floats to ints. Above assumption may not be true, but isn't that critical
            delay = int(delay)
            duration = int(duration)
            self.set(on, delay=delay)
            self.set(on, delay=delay + duration)
        else:
            super().pulse(duration)

    def get_new_telem(self) -> P31uLoadSwitchTelem:
        struct = self.driver.get_hk_out()  # query gomspace p31u for data

        # organize data
        self.telem = P31uLoadSwitchTelem(
            struct.output[self.p31u_output_id],
            struct.curout[self.p31u_output_id],
            struct.output_on_delta[self.p31u_output_id],
            struct.output_off_delta[self.p31u_output_id],
            struct.latchup[self.p31u_output_id],
        )
        return self.telem


class mockP31uLoadSwitch(P31uLoadSwitch):
    """Mocked loadswitch class. This loadswitch simulates the behavior of a real, hardware-connected
    loadswitch but purely in software. Useful for unit testing
    """

    def __init__(self, current_draw: int = 1800, variability: int = 100) -> None:
        self._state = False  # whether the loadswitch is on or off
        self.current_draw = current_draw  # mA
        self.variability = variability  # mA
        self.latchups: int = 0

    def _set(self, state: bool, delay: int = 0):
        time.sleep(delay)
        self._state = state

    def get_new_telem(self) -> P31uLoadSwitchTelem:
        """Since we have no hardware to get telemetry from, we generate some based off the loadswitch's state"""
        # adding in some variability to the measurement to be more realistic
        current_draw = self._state * int(
            self.current_draw + self.variability * random.uniform(-1, 1)
        )
        self.latchups += (
            random.random() < 0.01
        )  # latchups are single events effects that are (hopefully) quite rare
        return P31uLoadSwitchTelem(self._state, current_draw, 0, 0, self.latchups)


class lna(P31uLoadSwitch):
    """Loadswitch for the low-noise amplifier (LNA)"""

    p31u_output_id = GomOutputs.comms


class burnwire(P31uLoadSwitch):
    """Loadswitch for the separation mechanism's burn wire"""

    p31u_output_id = GomOutputs.burnwire_1


class glowplug_1(P31uLoadSwitch):
    """Loadswitch for the combustion chamber's first glowplug"""

    p31u_output_id = GomOutputs.glowplug_1


class glowplug_2(P31uLoadSwitch):
    """Loadswitch for the combustion chamber's second glowplug"""

    p31u_output_id = GomOutputs.glowplug_2


class solenoid(P31uLoadSwitch):
    """Loadswitch for the ACS system's solenoid valve.
    This loadswitch is significantly different from the other loadswitches
    because of the unique pulse characteristics required by the solenoid valve.
    Thus, when using with a real solenoid in the loop, use the `pulse` method rather than the `set` method.
    The `_set` method will still work, but should not be used to properly actuate the solenoid valve.
    LEAVING THIS ON FOR MORE THAN A FEW SECONDS WHILE A PHYSICAL SOLENOID VALVE IS CONNECTED WILL BREAK THE VALVE.
    DO NOT PLAN ON BREAKING A VALVE. THEY ARE LITERALLY IRREPLACEABLE (no longer manufactured and we have no spares)"""

    p31u_output_id = GomOutputs.solenoid

    def set(self, on: bool, delay: int = 0):
        msg = (
            "The solenoid valve cannot be actuated correctly using the set method."
            "If you want to actuate a physical solenoid valve, use the `pulse` method."
            "If you want to turn on the pin on the P31u without a solenoid in the loop, use the `_set` method"
        )

        if on:
            # If you try to turn on this load switch, error. Turns off just fine though.
            logging.critical(msg)
        else:
            self._set(on, delay=delay)

        self.get_new_telem()

    def pulse(self, duration: float):
        """Pulses the solenoid and holds it open for `duration` seconds
        The implementation of this will likely change soon
        because the solenoid driver circuit is probably changing."""

        if 0 < duration <= 3:
            self.driver.solenoid_single_wave(duration)
        else:
            logging.warning(
                "Invalid or unsafe solenoid valve pulse duration. You should NOT pulse the valve for more than 3 sec"
            )


class electrolyzers(P31uLoadSwitch):
    """Loadswitch for the propellant tank's electrolyzers"""

    p31u_output_id = GomOutputs.electrolyzer


class GPIOLoadSwitch(LoadSwitch):
    """Base loadswitch class for devices actuated solely by the GPIO pins on the RPi"""

    gpio_pin: int  # the pin number on the Raspberry pi, must be between 1-40

    def _set(self, state: bool):
        self.driver.set_gpio(self.gpio_pin, state)

    def get_new_telem(self) -> LoadSwitchTelem:
        state = self.driver.get_gpio(self.gpio_pin)
        self.telem = LoadSwitchTelem(state)
        return self.telem


class mockGPIOLoadSwitch(GPIOLoadSwitch):
    """Mocked loadswitch class. This loadswitch simulates the behavior of a real, hardware-connected
    GPIO loadswitch but purely in software. Useful for unit testing
    """

    def __init__(self):
        self._state = False

    def _set(self, state: bool, delay: int = 0):
        time.sleep(delay)
        self._state = state

    def get_new_telem(self) -> LoadSwitchTelem:
        return LoadSwitchTelem(self._state)


class power_amplifier(GPIOLoadSwitch):
    """Loadswitch for the RF power amplifier"""

    gpio_pin = PA_EN


class rf_switch_tx(GPIOLoadSwitch):
    """'Loadswitch' for the RF switch TX pin. Does not draw any load, but interfaces identically to a loadswitch"""

    gpio_pin = RF_TX_EN


class rf_switch_rx(GPIOLoadSwitch):
    """'Loadswitch' for the RF switch RX pin. Does not draw any load, but interfaces identically to a loadswitch"""

    gpio_pin = RF_RX_EN


# TODO: heater loadswitch?
# class heater(LoadSwitch):
#     def _set(self, state: bool):
#         self.driver.set_heater(0, 1, int(state))
#
#     def get_new_telem(self) -> LoadSwitchTelem:
#         state = self.driver.get_heater()
