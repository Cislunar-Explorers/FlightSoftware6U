import os
import sys
from threading import Thread
from time import sleep
from datetime import datetime
from queue import Queue
import signal
from utils.log import get_log
from json import load
from communications.satellite_radio import Radio

from dotenv import load_dotenv

import utils.constants
from utils.constants import *
from utils.parameters import *
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
#from OpticalNavigation.core import opnav
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
from communications.command_definitions import CommandDefinitions
from telemetry.telemetry import Telemetry


FOR_FLIGHT = None


class MainSatelliteThread(Thread):
    def __init__(self):
        super().__init__()
        
        self.init_parameters()
        self.command_queue = Queue()
        self.downlink_queue = Queue()
        self.commands_to_execute = []
        self.downlinks_to_execute = []
        self.telemetry = Telemetry(self)
        self.burn_queue = Queue()
        # self.init_comms()
        self.command_handler = CommandHandler()
        self.downlink_handler = DownlinkHandler()
        self.command_counter = 0
        self.downlink_counter = 0
        self.command_definitions = CommandDefinitions(self)
        self._init_sensors()
        self.last_opnav_run = datetime.now()  # Figure out what to set to for first opnav run
        self.log_dir = LOG_DIR
        self.logger = get_log()
        self.attach_sigint_handler()  # FIXME

        if os.path.isdir(self.log_dir):
            self.flight_mode = RestartMode(self)
        else:
            os.makedirs(CISLUNAR_BASE_DIR)
            os.mkdir(LOG_DIR)
            self.flight_mode = BootUpMode(self)
        self.create_session = create_sensor_tables_from_path(DB_FILE)
        #opnav.start()
        self.constants = utils.constants

    def init_comms(self):
        self.comms = CommunicationsSystem(
            queue=self.command_queue, use_ax5043=False
        )
        self.comms.listen()

    def init_parameters(self):
        with open(PARAMETERS_JSON_PATH) as f:
            json_parameter_dict = load(f)
        self.parameters = utils.parameters
            
        try:
            for parameter in self.parameters.__dir__():
                if parameter[0] != '_':
                    self.parameters.__setattr__(parameter,json_parameter_dict[parameter])
        except:
            raise Exception(
                'Attempted to set parameter ' + str(parameter) + 
                ', which could not be found in parameters.json'
            )

    # TODO

    def init_sensors(self):
        self.radio = Radio()
        self.gom = Gomspace()
        self.gyro = GyroSensor()
        self.adc = ADC()
        self.radio = Radio()
        self.rtc = RTC()
        # self.pressure_sensor = PressureSensor() # pass through self so need_to_burn boolean function
        # in pressure_sensor (to be made) can access burn queue"""

    def handle_sigint(self, signal, frame):
        self.shutdown()
        sys.exit(0)

    def attach_sigint_handler(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    # TODO (major: implement telemetry)
    def poll_inputs(self):
        
        #Telemetry downlink
        if (datetime.today() - self.radio.last_telemetry_time).total_seconds()/60 >= TELEM_DOWNLINK_TIME:
            self.enter_transmit_safe_mode()
            telemetry = self.command_definitions.gather_basic_telem()
            telem_downlink = (
                self.downlink_handler.pack_downlink(self.downlink_counter,FMEnum.Normal.value,NormalCommandEnum.BasicTelem.value,**telemetry))
            self.downlink_queue.put(telem_downlink)

        #Listening for new commands
        newCommand = self.radio.receiveSignal()
        if newCommand is not None:
            try:
                unpackedCommand = self.command_handler.unpack_command(newCommand)
                
                if unpackedCommand[0] == MAC:
                    if unpackedCommand[1] == self.command_counter + 1:
                        print('hello')
                        self.command_queue.put(bytes(newCommand))
                        self.command_counter+=1
                    else:
                        print('Command with Invalid Counter Received. '
                        + 'Counter: ' + str(unpackedCommand[1]))
                else:
                    print('Unauthenticated Command Received')

            except:
                print('Invalid Command Received')
        else:
            print('Not Received')

    def enter_transmit_safe_mode(self):
        #TODO: Make sure that everything else is turned off before transmitting
        pass

    def replace_flight_mode_by_id(self, new_flight_mode_id):
        self.replace_flight_mode(build_flight_mode(self, new_flight_mode_id))

    def replace_flight_mode(self, new_flight_mode):
        self.flight_mode = new_flight_mode
        self.logger.info(f"Changed to FM#{self.flight_mode.flight_mode_id}")

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
        self.flight_mode.execute_commands()

    def execute_downlinks(self):
        self.enter_transmit_safe_mode()
        while not self.downlink_queue.empty():
            self.radio.transmit(self.downlink_queue.get())
            self.downlink_counter += 1
            sleep(DOWNLINK_BUFFER_TIME)

    def read_command_queue_from_file(self, filename="communications/command_queue.txt"):
        # check if file exists
        if os.path.isfile(filename):
            text_file = open(filename, "r")
            for hex_line in text_file:
                self.command_queue.put(bytes.fromhex(hex_line))

            text_file.close()
            # delete file
            os.remove(filename)

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
                # self.update_state()
                #self.read_command_queue_from_file()
                self.execute_commands()  # Set goal or execute command immediately
                self.execute_downlinks()
                self.run_mode()
        finally:
            # TODO handle failure gracefully
            if FOR_FLIGHT is True:
                self.run()

    def shutdown(self):
        print("Shutting down...")
        # self.comms.stop()


if __name__ == "__main__":
    load_dotenv()
    FOR_FLIGHT = os.getenv("FOR_FLIGHT") == "FLIGHT"
    main = MainSatelliteThread()
    main.run()
