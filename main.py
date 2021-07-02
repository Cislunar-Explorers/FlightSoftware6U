import sys
from threading import Thread
from multiprocessing import Process
from time import time
from datetime import datetime
from queue import Queue
import signal
from utils.log import get_log, log_error
from typing import Optional

from json import load
from time import sleep

import utils.constants
from utils.constants import *
import utils.parameters as params
from utils.exceptions import CislunarException
from utils.db import create_sensor_tables_from_path

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
from utils.boot_cause import hard_boot

from communications.comms_driver import CommunicationsSystem

from communications.satellite_radio import Radio
from drivers.gom import Gomspace
from drivers.gyro import GyroSensor
from drivers.ADCDriver import ADC
from drivers.rtc import RTC
from drivers.nemo.nemo_manager import NemoManager
import OpticalNavigation.core.camera as camera

FOR_FLIGHT = None

logger = get_log()


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__()
        logger.info("Initializing...")
        self.init_parameters()
        self.command_queue = Queue()
        self.downlink_queue = Queue()
        self.FMQueue = Queue()
        self.commands_to_execute = []
        self.downlinks_to_execute = []
        self.telemetry = Telemetry(self)
        self.burn_queue = Queue()
        self.reorientation_queue = Queue()
        self.reorientation_list = []
        self.maneuver_queue = Queue()  # maneuver queue
        self.opnav_queue = Queue()  # determine state of opnav success
        # self.init_comms()
        self.command_handler = CommandHandler()
        self.downlink_handler = DownlinkHandler()
        self.command_counter = 0
        self.downlink_counter = 0
        self.command_definitions = CommandDefinitions(self)
        self.last_opnav_run = datetime.now()  # Figure out what to set to for first opnav run
        self.log_dir = LOG_DIR
        self.logger = get_log()
        self.attach_sigint_handler()  # FIXME
        self.file_block_bank = {}
        self.need_to_reboot = False

        self.gom: Optional[Gomspace] = None
        self.gyro: Optional[GyroSensor] = None
        self.adc: Optional[ADC] = None
        self.rtc: Optional[RTC] = None
        self.radio: Optional[Radio] = None
        self.mux: Optional[camera.CameraMux] = None
        self.camera: Optional[camera.Camera] = None
        self.init_sensors()

        # Opnav subprocess variables
        self.opnav_proc_queue = Queue()
        self.opnav_process = Process()  # define the subprocess

        if os.path.isdir(self.log_dir):
            self.flight_mode = RestartMode(self)
        else:
            os.makedirs(CISLUNAR_BASE_DIR, exist_ok=True)
            os.mkdir(LOG_DIR)
            self.flight_mode = BootUpMode(self)
        self.create_session = create_sensor_tables_from_path(DB_FILE)
        self.tlm = Telemetry(self)

    def init_comms(self):
        self.comms = CommunicationsSystem(
            queue=self.command_queue, use_ax5043=False
        )
        self.comms.listen()

    def init_parameters(self):
        try:
            with open(PARAMETERS_JSON_PATH) as f:
                json_parameter_dict = load(f)

            try:
                for parameter in utils.parameters.__dir__():
                    if parameter[0] != '_':
                        utils.parameters.__setattr__(parameter, json_parameter_dict[parameter])
            except:
                raise CislunarException(
                    f'Attempted to set parameter ' + str(parameter) +
                    ', which could not be found in parameters.json'
                )
        except Exception as e:
            log_error(e)

    def init_sensors(self) -> int:
        try:
            self.gom = Gomspace()
        except Exception as e:
            self.gom = None
            log_error(e)
            logger.error("GOM initialization failed")
        else:
            logger.info("Gom initialized")

        try:
            self.gyro = GyroSensor()
        except Exception as e:
            self.gyro = None
            log_error(e)
            logger.error("GYRO initialization failed")
        else:
            logger.info("Gyro initialized")

        try:
            self.adc = ADC(self.gyro)
            self.adc.read_temperature()
        except Exception as e:
            self.adc = None
            log_error(e)
            logger.error("ADC initialization failed")
        else:
            logger.info("ADC initialized")

        try:
            self.rtc = RTC()
            self.rtc.get_time()
        except Exception as e:
            self.rtc = None
            log_error(e)
            logger.error("RTC initialization failed")
        else:
            logger.info("RTC initialized")

        try:
            self.nemo_manager = NemoManager(port_id=3, data_dir=NEMO_DIR, reset_gpio_ch=16)
        except Exception as e:
            self.nemo_manager = None
            log_error(e)
            logger.error("NEMO initialization failed")
        else:
            logger.info("NEMO initialized")

        try:
            self.radio = Radio()
        except Exception as e:
            self.radio = None
            log_error(e)
            logger.error("RADIO initialization failed")
        else:
            logger.info("Radio initialized")

        # initialize the Mux, select a camera
        try:
            self.mux = camera.CameraMux()
            self.mux.selectCamera(1)
        except Exception as e:
            self.mux = None
            log_error(e)
            logger.error("MUX initialization failed")
        else:
            logger.info("Mux initialized")

        cameras_list = [0, 0, 0]

        # initialize cameras only if not a hard boot (or first boot)
        if not hard_boot() and os.path.isdir(self.log_dir):
            try:
                self.camera = camera.Camera()
                for i in [1, 2, 3]:
                    try:
                        self.mux.selectCamera(i)
                        f, t = self.camera.rawObservation(f"initialization-{i}-{int(time())}")
                    except Exception as e:
                        log_error(e)
                        logger.error(f"CAM{i} initialization failed")
                        cameras_list[i - 1] = 0
                    else:
                        logger.info(f"Cam{i} initialized")
                        cameras_list[i - 1] = 1

                if 0 in cameras_list:
                    raise Exception
            except Exception:
                self.camera = None
                logger.error(f"Cameras initialization failed")
            else:
                logger.info("Cameras initialized")
        else:
            self.need_to_reboot = True

        # make a bitmask of the initialized sensors for downlinking
        sensors = [self.gom, self.radio, self.gyro, self.adc, self.rtc, self.mux, self.camera]
        sensor_functioning_list = [int(bool(sensor)) for sensor in sensors]
        sensor_functioning_list.extend(cameras_list)
        sensor_bitmask = ''.join(map(str, sensor_functioning_list))
        logger.debug(f"Sensors: {sensor_bitmask}")
        return int(sensor_bitmask, 2)

    def handle_sigint(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def attach_sigint_handler(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    def poll_inputs(self):

        self.flight_mode.poll_inputs()
        # TODO: move this following if block to the telemetry file
        if self.radio is not None:
            # Telemetry downlink
            if (datetime.today() - self.radio.last_telemetry_time).total_seconds() / 60 >= params.TELEM_DOWNLINK_TIME:
                telemetry = self.command_definitions.gather_basic_telem()
                telem_downlink = (
                    self.downlink_handler.pack_downlink(self.downlink_counter, FMEnum.Normal.value,
                                                        NormalCommandEnum.BasicTelem.value, **telemetry))
                self.downlink_queue.put(telem_downlink)
                self.radio.last_telemetry_time = datetime.today()

            # Listening for new commands
            newCommand = self.radio.receiveSignal()
            if newCommand is not None:
                try:
                    unpackedCommand = self.command_handler.unpack_command(newCommand)

                    if unpackedCommand[0] == MAC:
                        if unpackedCommand[1] == self.command_counter + 1:
                            self.command_queue.put(bytes(newCommand))
                            self.command_counter += 1
                        else:
                            logger.warning('Command with Invalid Counter Received. Counter: ' + str(unpackedCommand[1]))
                    else:
                        logger.warning('Unauthenticated Command Received')
                except:
                    logger.error('Invalid Command Received')
            else:
                logger.debug('Not Received')

    def replace_flight_mode_by_id(self, new_flight_mode_id):
        self.replace_flight_mode(build_flight_mode(self, new_flight_mode_id))

    def replace_flight_mode(self, new_flight_mode):
        self.flight_mode = new_flight_mode
        self.logger.info(f"Changed to FM#{self.flight_mode.flight_mode_id}")

    def update_state(self):
        fm_to_update_to = self.flight_mode.update_state()

        # only replace the current flight mode if it needs to change (i.e. dont fix it if it aint broken!)
        if fm_to_update_to is None:
            pass
        elif fm_to_update_to != NO_FM_CHANGE and fm_to_update_to != self.flight_mode.flight_mode_id:
            self.replace_flight_mode_by_id(fm_to_update_to)

    def clear_command_queue(self):
        while not self.command_queue.empty():
            command = self.command_queue.get()
            logger.debug(f"Throwing away command: {command}")

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
        """This is the main loop of the Cislunar Explorers FSW and runs constantly during flight."""
        try:
            while True:
                sleep(2)  # TODO remove when flight modes execute real tasks

                self.poll_inputs()
                self.update_state()
                self.read_command_queue_from_file()
                self.execute_commands()  # Set goal or execute command immediately
                self.run_mode()
        except Exception as e:
            log_error(e, exc_info=1)
            logger.error("Error in main loop. Transitioning to SAFE mode")
        finally:
            # TODO handle failure gracefully
            if FOR_FLIGHT is True:
                self.replace_flight_mode_by_id(FMEnum.Safety.value)
                self.run()
            else:
                self.shutdown()

    def shutdown(self):
        if self.gom is not None:
            self.gom.all_off()
        if self.nemo_manager is not None:
            self.nemo_manager.close()
        logger.critical("Shutting down flight software")
        # self.comms.stop()

if __name__ == "__main__":
    FOR_FLIGHT = False
    main = MainSatelliteThread()
    main.run()
