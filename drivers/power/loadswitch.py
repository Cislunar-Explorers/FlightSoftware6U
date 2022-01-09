from drivers.power.power_controller import PA_EN, RF_RX_EN, RF_TX_EN, Power
from abc import ABC, abstractmethod
import time
from dataclasses import dataclass

from utils.constants import GomOutputs


@dataclass
class LoadSwitchTelem:
    state: bool


@dataclass
class P31uLoadSwitchTelem(LoadSwitchTelem):
    current_draw: int
    time_to_on: int
    time_to_off: int
    latchups: int


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

    def bounce(self, downtime=0.1):
        """If the loadswitch is on, flips the loadswitch off and back on"""
        if self.telem.state:
            self.set(False)
            time.sleep(downtime)
            self.set(True)

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


class P31uLoadSwitch(LoadSwitch):
    """Loadswitch for devices connected to the Gomspace P31u"""

    p31u_output_id: GomOutputs

    def _set(self, state: bool, delay: int = 0):
        self.driver.set_single_output(self.p31u_output_id, int(state), delay)

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
    The `set` method will still work, but should not be used to properly actuate the solenoid valve."""

    p31u_output_id = GomOutputs.solenoid

    def pulse(self):
        """The implementation of this will likely change soon
        because the solenoid driver circuit is probably changing."""
        self.driver.solenoid_single_wave(0.1)


class electrolyzers(P31uLoadSwitch):
    """Loadswitch for the propellant tank's electrolyzers"""

    p31u_output_id = GomOutputs.electrolyzer


class GPIOLoadSwitch(LoadSwitch):
    """Base loadswitch class for devices actuated solely by the GPIO pins on the RPi"""

    gpio_pin: int

    def _set(self, state: bool):
        self.driver.set_gpio(self.gpio_pin, state)

    def get_new_telem(self) -> LoadSwitchTelem:
        state = self.driver.get_gpio(self.gpio_pin)
        self.telem = LoadSwitchTelem(state)
        return self.telem


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
