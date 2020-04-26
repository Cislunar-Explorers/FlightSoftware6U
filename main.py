import os
import sys
from threading import Thread
from time import sleep
from queue import Queue
import signal

from utils.constants import (
    LOG_DIR,
    CISLUNAR_BASE_DIR,
    DB_FILE,
)
from utils.db import create_sensor_tables_from_path
from communications.comms_driver import CommunicationsSystem
from drivers.gom import Gomspace
from drivers.dummy_sensors import PressureSensor
from flight_modes.flight_mode import (
    RestartMode,
    BootUpMode,
)
from flight_modes.flight_mode_factory import build_flight_mode


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__()
        self.command_queue = Queue()
        self.commands_to_execute = []
        self.init_comms()
        self.init_sensors()
        self.log_dir = LOG_DIR
        self.attach_sigint_handler()  # FIXME
        if os.path.isdir(self.log_dir):
            self.flight_mode = RestartMode(self)
        else:
            self.flight_mode = BootUpMode(self)
        if not os.path.isdir(CISLUNAR_BASE_DIR):
            os.makedirs(CISLUNAR_BASE_DIR)
        self.create_session = create_sensor_tables_from_path(DB_FILE)

    def init_comms(self):
        self.comms = CommunicationsSystem(queue=self.command_queue, use_ax5043=False)  # noqa E501
        self.comms.listen()

    # TODO
    def init_sensors(self):
        self.gom = Gomspace()
        self.pressure_sensor = PressureSensor()

    def handle_sigint(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def attach_sigint_handler(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    # TODO
    def poll_inputs(self):
        pass

    def replace_flight_mode_by_id(self, new_flight_mode_id):
        self.flight_mode = build_flight_mode(self, new_flight_mode_id)

    def replace_flight_mode(self, new_flight_mode):
        self.flight_mode = new_flight_mode

    def update_state(self):
        self.flight_mode.update_state()

    def clear_command_queue(self):
        while not self.command_queue.empty():
            command = self.command_queue.get()
            print(f"Throwing away command: {command}")

    def reset_commands_to_execute(self):
        self.commands_to_execute = []

    # Execute received commands
    def execute_commands(self):
        assert len(self.commands_to_execute) == 0, "Didn't finish executing previous commands"
        while not self.command_queue.empty():
            self.commands_to_execute.append(self.command_queue.get())
        self.flight_mode.execute_current_commands()

    # Run the current flight mode
    # TODO ensure comms thread halts during realtime ops
    def run_mode(self):
        with self.flight_mode:
            self.flight_mode.run_mode()

    # Wrap in try finally block to ensure it stays live
    def run(self):
        try:
            while True:
                sleep(5)  # TODO remove when flight modes execute real tasks
                self.poll_inputs()
                self.update_state()
                self.execute_commands()  # Set goal or execute command immediately
                self.run_mode()
        finally:
            # TODO
            if False:
                self.run()

    def shutdown(self):
        print("Shutting down...")
        self.comms.stop()


if __name__ == "__main__":
    main = MainSatelliteThread()
    main.run()
