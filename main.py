from threading import Thread
from drivers.gom import Gomspace
from flight_mode import SafeMode, NormalMode

class SatelliteThread(Thread):
    def __init__(self):
        super().__init__(self)
        self.mode = NormalMode()

    # Read inputs and potentially change flight mode
    def poll_inputs(self):
        pass

    # Execute received commands
    def execute_commands(self):
        pass

    # Run the current flight mode
    def run_mode(self):
        self.mode.run_mode()

    def run(self):
        while True:
            self.poll_inputs()
            self.execute_commands()
            self.run_mode()


if __name__ == "__main__":
    main = SatelliteThread()
    main.run()