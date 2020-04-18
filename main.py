from threading import Thread
from queue import Queue

from constants import LOG_DIR
from drivers.communications import CommunicationsSystem
from flight_mode import NormalMode


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__(self)
        self.command_queue = Queue()
        self.comms = CommunicationsSystem(
            queue=self.command_queue, use_ax5043=False
        )  # noqa E501
        self.comms.listen()
        self.log_dir = LOG_DIR
        # TODO add additional conditions for handling the restart
        if os.isdir(self.log_dir):
            self.mode = RestartMode(self)
        else:
            self.mode = BootUpMode(self)
        # TODO initialize sensors

    # Read inputs
    # TODO
    def poll_inputs(self):
        pass

    # TODO implement logic for state transition
    def update_state(self):
        pass

    # Execute received commands
    def execute_commands(self):
        pass

    # Run the current flight mode
    # TODO ensure comms thread halts during realtime ops
    def run_mode(self):
        with self.mode:
            self.mode.run_mode()

    # Wrap in try finally block to ensure it stays live
    def run(self):
        while True:
            self.poll_inputs()
            self.execute_commands()  # Set goal or execute command immediately
            self.update_state()
            self.run_mode()

    def shutdown(self):
        self.comms.stop()


if __name__ == "__main__":
    main = MainSatelliteThread()
    main.run()
