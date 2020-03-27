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
