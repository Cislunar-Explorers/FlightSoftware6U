from threading import Thread
from drivers.communications import CommunicationsSystem
from flight_mode import NormalMode
from queue import Queue


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__(self)
        self.mode = NormalMode()
        self.command_queue = Queue()
        self.comms = CommunicationsSystem(
            queue=self.command_queue, use_ax5043=False
        )  # noqa E501
        self.comms.listen()
        # TODO initialize sensors

    # Read inputs and potentially change flight mode
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
            # XXX which should come first update state or execute commands
            self.update_state()
            self.execute_commands()
            self.run_mode()

    def shutdown(self):
        self.comms.stop()


if __name__ == "__main__":
    main = MainSatelliteThread()
    main.run()
