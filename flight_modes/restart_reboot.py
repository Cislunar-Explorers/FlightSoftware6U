import time
from datetime import datetime
# import os
# import sys

from utils.db import create_sensor_tables_from_path
from utils.constants import DB_FILE
from utils.log import get_log
from utils.db import RebootsModel

# these imports are necessary for MuxManager (camera init)
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_bus_device.i2c_device import I2CDevice
from picamera import PiCamera

# creates the session
# am I doing this right? is this session going to remain the same
# ... for the entire flight? i want to make sure it's adding to
# ... the same session always
create_session = create_sensor_tables_from_path(DB_FILE)
session = create_session()


class BootUpMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

        # wait a minute
        time.sleep(60)

        # initialize the spin
        self.init_spin()

        # add info about bootup to database
        self.log()

        # do the run_mode function inside flight_mode
        # no clue what this is really doing
        super().run_mode(self)

        # initialize the camera
        self.init_camera()

    # TODO make set of initial thrusts to achieve spin
    def init_spin(self):
        pass

    # ignore this for now, idk if it is necessary or what it is
    # def init_add_to_log(self):
    # make initial log entry
    # logger = get_log()

    # initialize the cameras, select a camera
    def init_camera(self):
        # create a MuxManager object
        mux = MuxManager()

        # this hopefully should select a camera
        mux.pick()

        # this should add to the RebootsModel

    # will add info about the bootup to the db
    def log(self):
        type_of_reboot = "Bootup"
        reboot_at = datetime.now()
        new_bootup = RebootsModel(type_of_reboot=type_of_reboot,
                                  reboot_at=reboot_at)
        session.add(new_bootup)
        session.commit()


class RestartMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

        # add info about restart to database
        self.log()

        # do the run_mode function inside flight_mode
        # no clue what this is really doing
        super().run_mode(self)

        # double check that the cameras are good
        self.init_camera()

    # TODO
    def add_to_log(self):
        # make a log entry 
        logger = get_log()

    # this should add to the RebootsModel
    # will add info about the bootup to the db
    def log(self):
        type_of_reboot = "Restart"
        reboot_at = datetime.now()
        new_bootup = RebootsModel(type_of_reboot=type_of_reboot,
                                  reboot_at=reboot_at)
        session.add(new_bootup)
        session.commit()

    # initialize the cameras, select a camera
    def init_camera(self):
        # create a MuxManager object
        mux = MuxManager()

        # this hopefully should select a camera
        mux.pick()

# this is the MuxManager object
# created to initailize the cameras
# this almost entirely comes from mux-test.py
class MuxManager:
    def __init__(self):

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.mux = I2CDevice(i2c, 0x70)

        self.sel_pin = DigitalInOut(board.D4)
        self.sel_pin.direction = Direction.OUTPUT
        self.oe1_pin = DigitalInOut(board.D17)
        self.oe1_pin.direction = Direction.OUTPUT
        self.oe2_pin = DigitalInOut(board.D18)
        self.oe2_pin.direction = Direction.OUTPUT

    # i wrote this, it pretty much tries to pick a camera
    def pick(self):
        for id in [1, 3, 4]:
            selectCamera(id)

    def selectCamera(self, id):
        if id == 1:
            with mux:
                mux.write(bytes([0x04]))
            sel_pin.value = False
            oe1_pin.value = False
            oe2_pin.value = True
        elif id == 2:
            with mux:
                mux.write(bytes([0x05]))
            sel_pin.value = True
            oe1_pin.value = False
            oe2_pin.value = True
        elif id == 3:
            with mux:
                mux.write(bytes([0x06]))
            sel_pin.value = False
            oe1_pin.value = True
            oe2_pin.value = False
        elif id == 4:
            with mux:
                mux.write(bytes([0x07]))
            sel_pin.value = True
            oe1_pin.value = True
            oe2_pin.value = False
        else:
            assert False
