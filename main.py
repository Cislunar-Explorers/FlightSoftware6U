from multiprocessing import Process
from queue import Queue, PriorityQueue
from threading import Thread
from time import sleep, time
from typing import Optional, List
import os
import signal
from flight_modes.flight_mode import FlightMode, TestMode
import logging
from time import sleep
import sys
import utils.constants as consts

# from drivers.dummy_sensors import PressureSensor
from flight_modes.restart_reboot import RestartMode, BootUpMode
from flight_modes.flight_mode_factory import build_flight_mode
from communications.command_handler import CommandHandler
from telemetry.telemetry import Telemetry
from utils.boot_cause import hard_boot
from udp_client.client import Client

from communications.comms_driver import CommunicationsSystem
from drivers.devices import DeviceContainer
from drivers.nemo.nemo_manager import NemoManager
import opnav.core.camera as camera
from utils.parameter_utils import init_parameters
from utils.db import create_sensor_tables_from_path
from sim.sim_data import SimData


class MainSatelliteThread(Thread):
    flight_mode: FlightMode

    def __init__(self, is_sim_run=False):
        super().__init__()
        logging.info("Initializing...")
        init_parameters()
        self.command_queue: Queue[bytes] = Queue()
        self.downlink_queue: Queue[bytes] = Queue()
        self.FMQueue: Queue[int] = Queue()
        self.commands_to_execute: List[bytes] = []
        self.burn_queue = Queue()
        self.reorientation_queue = Queue()
        self.reorientation_list = []
        self.maneuver_queue = PriorityQueue()
        self.opnav_queue = Queue()  # determine state of opnav success
        logging.info("Initializing commands and downlinks")
        self.command_handler = CommandHandler(self)
        self.command_counter = 0
        self.downlink_counter = 0
        self.last_opnav_run = time()  # Figure out what to set to for first opnav run
        self.log_dir = consts.LOG_DIR
        self.attach_sigint_handler()  # FIXME
        self.file_block_bank = {}
        self.need_to_reboot = False

        self.devices = DeviceContainer()
        self.mux: Optional[camera.CameraMux] = None
        self.camera: Optional[camera.PiCam] = None
        self.nemo_manager: Optional[NemoManager] = None

        # Opnav subprocess variables
        self.opnav_proc_queue = Queue()
        self.opnav_process = Process()  # define the subprocess

        if consts.TEST:
            logging.info("We are in Test Mode")
            self.flight_mode = TestMode(self)
        else:
            if os.path.isdir(self.log_dir):
                logging.info("We are in Restart Mode")
                self.flight_mode = RestartMode(self)
            else:
                logging.info("We are in Bootup Mode")
                os.makedirs(consts.CISLUNAR_BASE_DIR, exist_ok=True)
                os.mkdir(consts.LOG_DIR)
                self.flight_mode = BootUpMode(self)
                self.need_to_reboot = True

        self.create_session = create_sensor_tables_from_path(consts.DB_FILE)
        logging.info("Initializing Telemetry")
        self.telemetry = Telemetry(self)

        logging.info("opening UDP client socket")
        self.client = Client("192.168.0.200", 3333)

        self.is_sim_run = is_sim_run

        if self.is_sim_run:
            logging.info("initializing sim data object")
            self.sim_output = SimData()

        logging.info("Done intializing")

    def init_comms(self):
        """Deprecated. Not Used."""
        self.comms = CommunicationsSystem(queue=self.command_queue, use_ax5043=False)
        self.comms.listen()

    def init_sensors(self) -> int:

        connected_devices = self.devices.connect()

        try:
            self.nemo_manager = NemoManager(
                port_id=3, data_dir=consts.NEMO_DIR, reset_gpio_ch=16
            )
        except Exception as e:

            logging.error(e)
            logging.error("NEMO initialization failed")
        else:
            logging.info("NEMO initialized")

        # initialize the Mux, select a camera
        try:
            self.mux = camera.CameraMux()
            self.mux.selectCamera(1)
        except Exception as e:

            logging.error(e)
            logging.error("MUX initialization failed")
        else:
            logging.info("Mux initialized")

        cameras_list = [0, 0, 0]

        # initialize cameras only if not a hard boot (or first boot)
        if not hard_boot() and os.path.isdir(self.log_dir):
            try:
                self.camera = camera.PiCam()
                for i in [1, 2, 3]:
                    try:
                        self.mux.selectCamera(i)
                        f, t = self.camera.rawObservation(
                            f"initialization-{i}-{int(time())}"
                        )
                    except Exception as e:
                        logging.error(e)
                        logging.error(f"CAM{i} initialization failed")
                        cameras_list[i - 1] = 0
                    else:
                        logging.info(f"Cam{i} initialized with {f}:{t}")
                        cameras_list[i - 1] = 1

                if 0 in cameras_list:
                    raise Exception
            except Exception:

                logging.error("Cameras initialization failed")
            else:
                logging.info("Cameras initialized")
        else:
            self.need_to_reboot = True

        # make a bitmask of the initialized sensors for downlinking
        sensors = [self.mux, self.camera]
        sensor_functioning_list = [int(bool(sensor)) for sensor in sensors]
        sensor_functioning_list.extend(cameras_list)
        sensor_functioning_list.extend(map(int, connected_devices.values()))
        sensor_bitmask = "".join(map(str, sensor_functioning_list))
        logging.debug(f"Sensors: {sensor_bitmask}")
        return int(sensor_bitmask, 2)

    def handle_sigint(self, signal, frame):
        # TODO: add documentation (purpose of method?)
        self.shutdown()
        sys.exit(0)

    def attach_sigint_handler(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    def poll_inputs(self, sim_input=None):
        self.flight_mode.poll_inputs(sim_input)
        if sim_input is None:
            # TODO: move this following if block to the telemetry module
            if self.devices.radio.connected:
                # Listening for new commands
                newCommand = self.devices.radio.receiveSignal()
                if newCommand is not None:
                    self.command_queue.put(bytes(newCommand))
                else:
                    logging.debug("Not Received")
        else:
            # TODO: Sim out radio
            pass

    def replace_flight_mode_by_id(self, new_flight_mode_id):
        self.replace_flight_mode(build_flight_mode(self, new_flight_mode_id))

    def replace_flight_mode(self, new_flight_mode):
        self.flight_mode = new_flight_mode
        logging.info(f"Changed to FM#{self.flight_mode.flight_mode_id}")

    def update_state(self):
        fm_to_update_to = self.flight_mode.update_state(self.sim_output)

        # only replace the current flight mode if it needs to change (i.e. dont fix it if it aint broken!)
        if fm_to_update_to is None:
            pass
        elif (
            fm_to_update_to != consts.NO_FM_CHANGE
            and fm_to_update_to != self.flight_mode.flight_mode_id
        ):
            self.replace_flight_mode_by_id(fm_to_update_to)
        return self.flight_mode

    def clear_command_queue(self):
        while not self.command_queue.empty():
            command = self.command_queue.get()
            logging.debug(f"Throwing away command: {command}")

    def reset_commands_to_execute(self):
        self.commands_to_execute = []

    # Execute received commands
    def execute_commands(self):
        assert (
            len(self.commands_to_execute) == 0
        ), "Didn't finish executing previous commands"
        while not self.command_queue.empty():
            self.commands_to_execute.append(self.command_queue.get())
        self.flight_mode.execute_commands()

    def read_command_queue_from_file(self, filename="communications/command_queue.txt"):
        """A temporary workaround to not having radio board access"""
        # check if file exists
        if os.path.isfile(filename):
            text_file = open(filename, "r")
            for hex_line in text_file:
                self.command_queue.put(bytes.fromhex(hex_line))

            text_file.close()
            # delete file
            os.remove(filename)

    def read_command_queue_from_sim(self, sim_input):
        # TODO: Add sim_input commands to self.comamnd_queue
        pass

    # Run the current flight mode
    def run_mode(self):
        with self.flight_mode:
            self.flight_mode.run_mode()

    # Step through one time step
    def step(self, sim_input):
        self.sim_input = sim_input
        self.poll_inputs(sim_input)

        # Write updated state per time step to output
        updated_state = self.update_state()
        self.sim_output.write_multi_entries(updated_state.__dict__)

        # Read from sim input command queue
        self.read_command_queue_from_sim(sim_input)
        self.execute_commands()  # Set goal or execute command immediately
        self.run_mode()

        # write data to sim data object
        self.sim_output.write_multi_entries(self.telemetry.detailed_packet_dict())
        return self.sim_output.to_dict()

    # Wrap in try finally block to ensure it stays live
    def run(self):
        """This is the main loop of the Cislunar Explorers FSW and runs constantly during flight."""
        self.init_sensors()
        try:
            while True:
                sleep(2)  # TODO remove when flight modes execute real tasks

                self.poll_inputs()
                self.update_state()
                self.read_command_queue_from_file()
                self.execute_commands()  # Set goal or execute command immediately
                self.run_mode()

                # send data udp
                self.client.send_data(self.telemetry.detailed_packet_dict())

        except (Exception, SystemExit) as e:
            logging.error(e, exc_info=True)
            logging.error("Error in main loop. Transitioning to SAFE mode")
        finally:
            # TODO handle failure gracefully
            if consts.FOR_FLIGHT:
                self.replace_flight_mode_by_id(consts.FMEnum.Safety.value)
                self.run()
            else:
                self.shutdown()

    def shutdown(self):
        if self.sim_input is None:
            if self.devices.gom.connected:
                self.devices.gom.all_off()
            if self.nemo_manager is not None:
                self.nemo_manager.close()
        if self.sim_output is None:
            logging.critical("Shutting down flight software")
        else:
            self.sim_output.write_single_entry("Shutting down flight software", "")


if __name__ == "__main__":
    main = MainSatelliteThread()
    main.run()
