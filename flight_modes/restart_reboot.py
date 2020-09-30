# things to import:
# time
# os
# from utils.db import create_sensor_tables_from_path
# other stuff too?




"""
class BootUpMode( param? ):
    def __init__(self):

        #do i need to assign any variables? i don't think so but not sure
       

        self.wait_a_minute()
        self.create_database()
        self.init_spin(this one actually might need some parameters)
        self.init_add_to_log()
        self.init_camera()
        self.create_record_of_restarts()


        self.create_session = create_sensor_tables_from_path( something )



    def wait_a_minute(self):
        # wait 1 minute before doing anything
        # we don't want to blow up NASA

    
    def create_database(self):
        # create a database for log entry text files
        log_entries = []


    def init_spin(params):
        # make set of thrusts to achieve initial spin
        # i currently have zero clue how to do this


    def init_add_to_log(self):
        # make initial log entry to log_entries


    def init_camera(self):
        # ask stephen what to do here
        # camera multiplexer needs to be initialized


    def create_record_of_restarts(self):
        # creates a blank record (dictionary maybe?)
        # for future restarts




classRestartMode( param? ):
    def __init__(self):

        #do i need to assign any variables? i don't think so but not sure
   

        self.add_to_log()
        self.add_to_restart_records()
        self.write_to_i2c()



    def add_to_log(self):
        # make a log entry text file


    def add_to_restart_records(self)
        # record what number restart this is, and maybe the time?
        # add this to the record created in BootUp


    def write_to_i2c(self):
        # i have literally no idea what this means


"""
