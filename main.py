from threading import Thread
from queue import Queue

from utils.constants import (
    LOG_DIR,
    FlightModeEnum,
    ENTER_LOW_BATTERY_MODE_THRESHOLD,
    EXIT_LOW_BATTERY_MODE_THRESHOLD,
    CISLUNAR_BASE_DIR,
    DB_FILE,
)
from utils.db import create_sensor_tables_from_path
from drivers.communications import CommunicationsSystem
from drivers.gom import Gomspace
from drivers.dummy_sensors import PressureSensor
from flight_modes.flight_mode import (
    NormalMode,
    ManeuverMode,
    ElectrolysisMode,
    RestartMode,
    BootUpMode,
)
from flight_modes.opnav_flightmode import OpNavMode


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__(self)
        self.command_queue = Queue()
        self.init_comms()
        self.init_sensors()
        self.log_dir = LOG_DIR
        if os.isdir(self.log_dir):
            self.mode = RestartMode(self)
        else:
            self.mode = BootUpMode(self)
        if not os.isdir(CISLUNAR_BASE_DIR):
            os.makedirs(CISLUNAR_BASE_DIR)
        self.create_session = create_sensor_tables_from_path(DB_FILE)

    def init_comms(self):
        self.comms = CommunicationsSystem(queue=self.command_queue, use_ax5043=False)
        self.comms.listen()

    # TODO
    def init_sensors(self):
        self.gom = Gomspace()
        self.pressure_sensor = PressureSensor()

    # TODO
    def poll_inputs(self):
        pass

    # TODO implement logic for state transition

    def replace_flight_mode(self, new_flight_mode):
        self.flight_mode = new_flight_mode

    def update_state(self):
        flight_mode_id = self.mode.flight_mode_id
        if flight_mode_id == FlightModeEnum.LowBatterySafety.value:
            if self.gom.read_battery_percentage() >= EXIT_LOW_BATTERY_MODE_THRESHOLD:
                self.flight_mode = NormalMode()
        elif flight_mode_id == FlightModeEnum.Normal.value:
            pass
            # TODO do I need to enter electrolysis to prepare for maneuver?
            # do I need to start a maneuver?
            # do I need to run OpNav?
        elif flight_mode_id == FlightModeEnum.Boot.value:
            pass  # NOTE: only exit BootUp mode on command from ground station
        elif flight_mode_id == FlightModeEnum.Electrolysis.value:
            if self.flight_mode.task_completed is True:
                elf.replace_flight_mode(ManeuverMode(self, 10))
        elif flight_mode_id == FlightModeEnum.Restart.value:
            if self.flight_mode.task_completed is True:
                elf.replace_flight_mode(NormalMode(self))
        elif flight_mode_id == FlightModeEnum.Maneuver.value:
            if self.flight_mode.task_completed is True:
                elf.replace_flight_mode(NormalMode(self))
        elif flight_mode_id == FlightModeEnum.OpNav.value:
            if self.flight_mode.task_completed is True:
                self.replace_flight_mode(NormalMode(self))
        else:
            raise NotImplementedError

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
            self.update_state()
            self.execute_commands()  # Set goal or execute command immediately
            self.run_mode()

    def shutdown(self):
        self.comms.stop()


if __name__ == "__main__":
    main = MainSatelliteThread()
    main.run()
