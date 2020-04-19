import gc

# Necessary modes to implement
# BootUp, Normal, Eclipse, Safety, Electrolysis, Propulsion,
# Attitude Adjustment, Transmitting, OpNav (image processing)
# TestModes


class FlightMode:
    def __init__(self, parent):
        self.parent = parent

    def run_mode(self):
        pass

    def read_sensors(self):
        pass

    # Autonomous actions to maintain safety
    def automatic_actions(self):
        pass

    def write_telemetry(self):
        pass


# Model for FlightModes that require precise timing
# Pause garbage collection and anything else that could
# interrupt critical thread
class PauseBackgroundMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)

    def __enter__(self):
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()


class SafeMode(FlightMode):
    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Execute safe mode")


class NormalMode(FlightMode):
    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Execute normal mode")

class ManeuverMode(FlightMode):

    #in km and km/s
    x = 0.0
    y = 0.0
    z = 0.0
    v_x = 0.0
    v_y = 0.0
    v_z = 0.0

    #Need a type for 

    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Executing Maneuver")
    
    def update_pos_and_vel(self):
        #TODO
        #Access database and obtain OpNav data.
        pass

    def pulse(self): #Maybe have some arguments if we can control the pulse force
        #TODO
        pass

    def adjust_attitude(self): #Pass in rotation arguments. Either a quarternion or euler angles
        #TODO
        pass
    
    def fire_cold_gas(self, ms): #Fire the cold gas thruster for [ms] milliseconds
        #TODO
        pass
