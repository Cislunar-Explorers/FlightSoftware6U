import power_controller as pc


class Gomspace:
    def __init__(self):
        self.gom = pc.Power()

    def tick_wdt(self):
        """Resets dedicated WDT"""
        return self.gom.reset_wdt()

    def get_health_data(self, level=None):
        """Returns a struct containing housekeeping data.
            The level parameter specifies which command gets sent to the P31u and what data you get back.
            level must be either one of the following: \n
            ["default", "eps", "vi", "out", "wdt", "basic", "config", "config2"]\n
            or the index of one of the above: i.e. get_health_data("eps") is the same as get_health_data(1)\n
            If no argument is provided, returns the same as "default" or 0\n
            Every option returns a different struct, the documentation for which can be found in power_structs.py or in
            the GomSpace P31u manual"""

        if level is None:
            return self.gom.get_hk_1()

        assert type(level) is str or type(level) is int
        option_index = 0

        if type(level) == str:
            options = ["default", "eps", "vi", "out", "wdt", "basic"]
            assert level.lower() in options
            option_index = options.index(level.lower())

        if type(level) == int:
            assert level in range(0, 6)
            option_index = level

        assert option_index in range(0, 6), "Something went wrong!"

        # there's probably a better way to do this:
        if option_index == 0:
            return self.gom.get_hk_1()
        elif option_index == 1:
            return self.gom.get_hk_2()
        elif option_index == 2:
            return self.gom.get_hk_2_vi()
        elif option_index == 3:
            return self.gom.get_hk_out()
        elif option_index == 4:
            return self.gom.get_hk_wdt()
        elif option_index == 5:
            return self.gom.get_hk_2_basic()

    def set_output(self, channel, value, delay=0):
        """Sets a single controllable output either on or off.
            channel must be between 1 and 6
            value must be either 1 (on) or 0 (off)"""
        self.gom.set_single_output(channel, value, delay)

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        self.gom.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        self.gom.hard_reset(passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        self.gom.displayAll()

    def solenoid(self, spike, hold, delay=0):
        """Spikes the solenoid at 20V for [spike] milliseconds, holds at 5V for [hold] milliseconds"""
        self.gom.solenoid(spike, hold, delay)
        self.electrolysis = False

    def glowplug(self, duration, delay=0):
        """Pulses the glowplug for [duration] milliseconds with after a delay of [delay] seconds"""
        self.gom.glowplug(duration, delay)

    def burnwires(self, duration, delay=0):
        """Turns on both burnwires for [duration] seconds after [delay] seconds. Does a display_all half way through"""
        self.gom.burnwire(duration, delay)

    # TODO
    def get_health_data(self):
        return {}

    def is_electrolyzing(self):
        return self.electrolysis

    def set_electrolysis(self, status: bool):
        self.electrolysis = status

    def read_battery_percentage(self):
        return 0.7
