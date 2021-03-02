import os
import sys
from threading import Thread
from time import sleep
from datetime import datetime
from queue import Queue
import signal
import random
from utils.log import get_log
import OpticalNavigation.core.camera as camera
from communications.satellite_radio import Radio

from utils.constants import (
    LOG_DIR,
    CISLUNAR_BASE_DIR,
    DB_FILE,
    NO_FM_CHANGE
)  # TODO: optimize this import

import utils.constants
from utils.db import create_sensor_tables_from_path
from communications.comms_driver import CommunicationsSystem
from drivers.gom import Gomspace
from drivers.gyro import GyroSensor
from drivers.ADCDriver import ADC
from drivers.rtc import RTC
# from drivers.dummy_sensors import PressureSensor
from flight_modes.restart_reboot import (
    RestartMode,
    BootUpMode,
)
from flight_modes.flight_mode_factory import build_flight_mode
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
from communications.command_definitions import CommandDefinitions
from telemetry.telemetry import Telemetry

FOR_FLIGHT = None

logger = get_log()


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__()
        logger.info("Initializing...")
        self.command_queue = Queue()
        self.communications_queue = Queue()
        self.FMQueue = Queue()
        self.commands_to_execute = []
        self.burn_queue = Queue()
        self.reorientation_queue = Queue()
        self.reorientation_list = []
        self.maneuver_queue = Queue()  # maneuver queue
        # self.init_comms()
        self.command_handler = CommandHandler()
        self.downlink_handler = DownlinkHandler()
        self.command_definitions = CommandDefinitions(self)
        self.last_opnav_run = datetime.now()  # Figure out what to set to for first opnav run
        self.log_dir = LOG_DIR
        self.logger = get_log()
        self.attach_sigint_handler()  # FIXME

        self.gom = None
        self.gyro = None
        self.adc = None
        self.rtc = None
        self.radio = None
        self.mux = None
        self.camera = None
        self.init_sensors()

        # Telemetry
        self.tlm = Telemetry(self)

        if os.path.isdir(self.log_dir):
            self.flight_mode = RestartMode(self)
        else:
            os.makedirs(CISLUNAR_BASE_DIR)
            os.mkdir(LOG_DIR)
            self.flight_mode = BootUpMode(self)
        self.create_session = create_sensor_tables_from_path(DB_FILE)
        self.constants = utils.constants

    def init_comms(self):
        self.comms = CommunicationsSystem(
            queue=self.command_queue, use_ax5043=False
        )
        self.comms.listen()

    # TODO

    def init_sensors(self):
        try:
            self.gom = Gomspace()
        except:
            self.gom = None
            logger.error("Gom initialization failed")
        else:
            logger.info("Gom initialized")

        try:
            self.gyro = GyroSensor()
        except:
            self.gyro = None
            logger.error("Gyro initialization failed")
        else:
            logger.info("Gyro initialized")

        try:
            self.adc = ADC(self.gyro)
            self.adc.read_temperature()
        except:
            self.adc = None
            logger.error("ADC initalization failed")
        else:
            logger.info("ADC initialized")

        try:
            self.radio = Radio()
        except:
            self.radio = None
            logger.error("Radio initialization failed")
        else:
            logger.info("Radio initialized")

        try:
            self.rtc = RTC()
        except:
            self.rtc = None
            logger.error("RTC initialization failed")
        else:
            logger.info("RTC initialized")

        # self.pressure_sensor = PressureSensor() # pass through self so need_to_burn boolean function
        # in pressure_sensor (to be made) can access burn queue"""

        # initialize the cameras, select a camera

        try:
            logger.info("Creating camera mux...")
            self.mux = camera.CameraMux()
            self.camera = camera.Camera()
            logger.info("Selecting a camera...")
            # select camera before reboot so that we will detect cam on reboot
            self.mux.selectCamera(random.choice([1, 2, 3]))
            logger.info("Taking a raw observation...")
            self.camera.rawObservation("restart_cam_test.mjpeg")
        except:
            self.mux = None
            self.camera = None
            logger.error("Camera initialization failed")
        else:
            logger.info("Cameras initialized")

    def handle_sigint(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def attach_sigint_handler(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    def poll_inputs(self):
        if self.radio is not None:
            newCommand = self.radio.receiveSignal()
            if newCommand is not None:
                try:
                    self.command_handler.unpack_command(newCommand)  # Only for error checking
                    self.command_queue.put(bytes(newCommand))
                except:
                    print('Invalid Command Received')
            else:
                print('Not Received')
        # self.tlm.poll()
        self.flight_mode.poll_inputs()

    def replace_flight_mode_by_id(self, new_flight_mode_id):
        self.replace_flight_mode(build_flight_mode(self, new_flight_mode_id))

    def replace_flight_mode(self, new_flight_mode):
        self.flight_mode = new_flight_mode
        self.logger.info(f"Changed to FM#{self.flight_mode.flight_mode_id}")

    def update_state(self):
        fm_to_update_to = self.flight_mode.update_state()

        # only replace the current flight mode if it needs to change (i.e. dont fix it if it aint broken!)
        if fm_to_update_to != NO_FM_CHANGE and fm_to_update_to != self.flight_mode.flight_mode_id:
            self.replace_flight_mode_by_id(fm_to_update_to)

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

    # Run the current flight mode
    def run_mode(self):
        with self.flight_mode:
            self.flight_mode.run_mode()

    # Wrap in try finally block to ensure it stays live
    def run(self):
        try:
            while True:
                # sleep(5)
                logger.info("flight mode: " + repr(self.flight_mode))
                self.poll_inputs()
                self.update_state()
                self.read_command_queue_from_file()
                self.execute_commands()  # Set goal or execute command immediately
                self.run_mode()
        finally:
            self.replace_flight_mode_by_id(utils.constants.FMEnum.Safety.value)
            # TODO handle failure gracefully
            if FOR_FLIGHT is True:
                self.run()

    def shutdown(self):
        self.gom.all_off()
        print("Shutting down...")
        # self.comms.stop()


if __name__ == "__main__":
    FOR_FLIGHT = False
    main = MainSatelliteThread()
    main.run()
