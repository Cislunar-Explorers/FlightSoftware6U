import os
import sys
from threading import Thread
from time import sleep
from datetime import datetime
from queue import Queue
import signal

from dotenv import load_dotenv

from utils.constants import (
    LOG_DIR,
    CISLUNAR_BASE_DIR,
    DB_FILE,
    LOW_CRACKING_PRESSURE,
    HIGH_CRACKING_PRESSURE,
    IDEAL_CRACKING_PRESSURE
)
from utils.db import create_sensor_tables_from_path
from communications.comms_driver import CommunicationsSystem
from drivers.gom import Gomspace
from drivers.dummy_sensors import PressureSensor
from flight_modes.restart_reboot import (
    RestartMode,
    BootUpMode,
)
from flight_modes.flight_mode_factory import build_flight_mode


FOR_FLIGHT = None


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__()
        self.command_queue = Queue()
        self.commands_to_execute = []
        self.burn_queue = Queue()
        self.init_comms()
        # self.init_sensors()  # TODO FIX THIS!
        self.last_opnav_run = datetime.now()  # Figure out what to set to for first opnav run
        self.log_dir = LOG_DIR
        self.attach_sigint_handler()  # FIXME
        if os.path.isdir(self.log_dir):
            self.flight_mode = RestartMode(self)
        else:
            os.makedirs(CISLUNAR_BASE_DIR)
            os.mkdir(LOG_DIR)
            self.flight_mode = BootUpMode(self)
        self.create_session = create_sensor_tables_from_path(DB_FILE)

    def init_comms(self):
        self.comms = CommunicationsSystem(
            queue=self.command_queue, use_ax5043=False
        )
        self.comms.listen()

    # TODO
    def init_sensors(self):
        self.gom = Gomspace()
        self.pressure_sensor = PressureSensor() # pass through self so need_to_burn boolean function
                                                # in pressure_sensor (to be made) can access burn queue"""
    def handle_sigint(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def attach_sigint_handler(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    # TODO
    def poll_inputs(self):
        # Switch on/off electrolyzer
        curr_pressure = self.pressure_sensor.read_pressure()
        if curr_pressure < IDEAL_CRACKING_PRESSURE:
            if not self.gom.is_electrolyzing():
                self.gom.set_electrolysis(True)
        else:
            self.gom.set_electrolysis(False)

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
        assert (
            len(self.commands_to_execute) == 0
        ), "Didn't finish executing previous commands"
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
            # TODO handle failure gracefully
            if FOR_FLIGHT is True:
                self.run()

    def shutdown(self):
        print("Shutting down...")
        self.comms.stop()


if __name__ == "__main__":
    load_dotenv()
    FOR_FLIGHT = os.getenv("FOR_FLIGHT") == "FLIGHT"
    main = MainSatelliteThread()
    main.run_mode()  # TODO going to actually be main.run()


