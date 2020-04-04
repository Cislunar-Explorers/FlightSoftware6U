import gc

# Necessary modes to implement
# BootUp, Normal, Eclipse, Safety, Electrolysis, Propulsion,
# Attitude Adjustment, Transmitting, OpNav (image processing)
# TestModes


class FlightMode:
    def __init__(self, parent):
        self.parent = parent

    def run_mode(self):
        pass

    def read_sensors(self):
        pass

    # Autonomous actions to maintain safety
    def automatic_actions(self):
        pass

    def write_telemetry(self):
        pass


# Model for FlightModes that require precise timing
# Pause garbage collection and anything else that could
# interrupt critical thread
class PauseBackgroundMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

    def __enter__(self):
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()


class SafeMode(FlightMode):
    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Execute safe mode")


class NormalMode(FlightMode):
    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Execute normal mode")
