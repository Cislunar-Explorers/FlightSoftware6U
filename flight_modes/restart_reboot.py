import time
from datetime import datetime
import os
import sys
from utils.db import create_sensor_tables_from_path 
from utils.constants import DB_FILE
from utils.log import get_log




# the number of times a reboot or bootup has happened
num_reboots = 0

# dictionary keeping track of number of reboots, and when they happen
reboots = {}

class BootUpMode:
    def __init__(self):
       
        self.wait_a_minute()
        self.init_spin()
        self.init_add_to_log()
        self.init_camera()
        self.add_to_restart_records()

        # create a session... i don't know if the parameter is right here
        self.create_session = create_sensor_tables_from_path(DB_FILE)


    def wait_a_minute(self):
        # waits 1 minute before doing anything
        time.sleep(60)


    def init_spin(self):
        # make set of thrusts to achieve initial spin
        # i currently have zero clue how to do this
        pass


    def init_add_to_log(self):
        # make initial log entry 
        logger = get_log()


    def init_camera(self):
        # camera multiplexer needs to be initialized
        pass


    def add_to_restart_records(self):
        # record that a reboot is happening, and the time it is happening
        reboots[num_reboots] = datetime.now()
        num_reboots += 1




class RestartMode:
    def __init__(self):

        #do i need to assign any variables? i don't think so but not sure
   

        self.add_to_log()
        self.add_to_restart_records()
        self.write_to_i2c()


    def add_to_log(self):
        # make a log entry 
        logger = get_log()


    def add_to_restart_records(self):
        # record what number restart this is, and maybe the time?
        # add this to the record created in BootUp
        reboots[num_reboots] = datetime.now()
        num_reboots += 1


    def write_to_i2c(self):
        # i have literally no idea what this means
        pass
