# power_controller.py
#
# Desc: Drivers for the GomSpace NanoPower P31u
#
# Author: Daniel Kim (dsk252)
# Revisited by: Tobias Fischer (tmf97) and Stephen Zakoworotny (sjz38) in Spring 2020
#
# Date: 10/3/16
#
# Status: Main functionality completed
#         Higher level functions need to 
#         be implemented (if necessary)

from pigpio import *
from power_structs import *
import RPi.GPIO as GPIO
import time

# I2C libraries
import Adafruit_ADS1x15
from SDL_DS3231 import SDL_DS3231
from L3GD20 import L3GD20

# pipeline operator (>>_>>)
_ = power_structs._

# power device address
POWER_ADDRESS           = 0x02

# raspberry pi bus number
PI_BUS                  = 1

# allowed ranges
MAX_PV_VOLTAGE			= 4000  # change later

# command registers
CMD_PING                = 0x01
CMD_REBOOT              = 0x04
CMD_GET_HK              = 0x08
CMD_SET_OUTPUT          = 0x09
CMD_SET_SINGLE_OUTPUT   = 0x0A
CMD_SET_PV_VOLT         = 0x0B
CMD_SET_PV_AUTO         = 0x0C
CMD_SET_HEATER          = 0x0D
CMD_RESET_COUNTERS      = 0x0F
CMD_RESET_WDT           = 0x10
CMD_CONFIG_CMD          = 0x11
CMD_CONFIG_GET          = 0x12
CMD_CONFIG_SET          = 0x13
CMD_HARD_RESET          = 0x14
CMD_CONFIG2_CMD         = 0x15
CMD_CONFIG2_GET         = 0x16
CMD_CONFIG2_SET         = 0x17

# struct sizes in bytes
SIZE_HKPARAM_T          = 44
SIZE_EPS_HK_T           = 136
SIZE_EPS_HK_VI_T        = 20
SIZE_EPS_HK_OUT_T       = 64
SIZE_EPS_HK_WDT_T       = 28
SIZE_EPS_HK_BASIC_T     = 24
SIZE_EPS_CONFIG_T       = 58
SIZE_EPS_CONFIG2_T      = 20

# power output channels
OUT_1                   = 0  # 5V
OUT_2                   = 1  # 5V
OUT_3                   = 2  # 5V
OUT_4                   = 3  # 5V
OUT_5                   = 4  # 5V
OUT_6                   = 5  # 3.3V
OUT_HEATER              = 6
OUT_SWITCH              = 7

# powered components
OUT_COMMS_AMP       = OUT_1
OUT_BURNWIRE_1      = OUT_2
OUT_BURNWIRE_2      = OUT_3
OUT_GLOWPLUG        = OUT_4
OUT_SOLENOID        = OUT_5
OUT_ELECTROLYZER    = OUT_6

# Outputs on board:
#
#       H1
#     .-----.
#  47 | 0 3 | 48
#  49 | 1 4 | 50
#  51 | 2 5 | 52
#     '-----'
#

# pi outputs
OUT_PI_COMMS            = 11    # GPIO 17
OUT_PI_SOLENOID_ENABLE  = 40    # GPIO 21


class Power(object):
    # initializes power object with bus [bus] and device address [addr]
    def __init__(self, bus=PI_BUS, addr=POWER_ADDRESS, flags=0):
        self._pi = pi()                                     # initialize pigpio object
        self._dev = self._pi.i2c_open(bus, addr, flags)     # initialize i2c device

        # I2C devices
        self._adc = Adafruit_ADS1x15.ADS1115()              # initialize adc
        self._rtc = SDL_DS3231.SDL_DS3231(1, 0x68)          # initialize rtc
        self._gyro = L3GD20(busId 		 = 1,            	# initialize gyro
                            slaveAddr 	 = 0x6b, 
                            ifLog 		 = False, 
                            ifWriteBlock = False)
        
        # Preconfiguration
        self._gyro.Set_PowerMode("Normal")
        self._gyro.Set_FullScale_Value("250dps")
        self._gyro.Set_AxisX_Enabled(True)
        self._gyro.Set_AxisY_Enabled(True)
        self._gyro.Set_AxisZ_Enabled(True)

        # Print current configuration
        self._gyro.Init()
        self._gyro.Calibrate()

        # initialize pi outputs
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(OUT_PI_SPARKPLUG, GPIO.OUT)
        GPIO.setup(OUT_PI_COMMS, GPIO.OUT)
        GPIO.setup(OUT_PI_SOLENOID_ENABLE, GPIO.OUT)
        GPIO.output(OUT_PI_SPARKPLUG, GPIO.HIGH)
        GPIO.output(OUT_PI_COMMS, GPIO.LOW)
        GPIO.output(OUT_PI_SOLENOID_ENABLE, GPIO.HIGH)

        # initialize eps outputs
        self.set_output(0)

    # prints housekeeping/config/config2
    def displayAll(self):
        displayHK(self.get_hk_1())
        displayConfig(self.config_get())
        displayConfig2(self.config2_get())

    # writes byte list [values] with command register [cmd]
    # raises: ValueError if cmd or values are not bytes
    def write(self, cmd, values):
        self._pi.i2c_write_device(self._dev, bytearray([cmd]+values))

    # reads [bytes] number of bytes from the device and returns a bytearray
    # This function does not currently return the error code of the i2c stream. Is this something that we want?
    def read(self, bytes):
        # first two read bytes -> [command][error code][data]
        (x, r) = self._pi.i2c_read_device(self._dev, bytes+2) 
        if r[1] != 0:
            print("Command %i failed with error code %i" % (r[0], r[1]))
        return r[2:]

    # Not sure what value is in the below function, need to get cleared up
    # pings value
    # value [1 byte]
    # raises: ValueError if value is not a byte
    def ping(self, value):
        self.write(CMD_PING, [value])
        return self.read(1)

    # reboot the system
    def reboot(self):
        self.write(CMD_REBOOT, [0x80, 0x07, 0x80, 0x07])

    # returns hkparam_t struct
    def get_hk_1(self):
        self.write(CMD_GET_HK, [])
        array = self.read(SIZE_HKPARAM_T)
        return c_bytesToStruct(array, "hkparam_t")

    # returns eps_hk_t struct
    def get_hk_2(self):
        self.write(CMD_GET_HK, [0x00])
        array = self.read(SIZE_EPS_HK_T)
        return c_bytesToStruct(array, "eps_hk_t")

    # returns eps_hk_vi_t struct
    def get_hk_2_vi(self):
        self.write(CMD_GET_HK, [0x01])
        array = self.read(SIZE_EPS_HK_VI_T)
        return c_bytesToStruct(array, "eps_hk_vi_t")

    # returns eps_hk_out_t struct
    def get_hk_out(self):
        self.write(CMD_GET_HK, [0x02])
        array = self.read(SIZE_EPS_HK_OUT_T)
        return c_bytesToStruct(array, "eps_hk_out_t")

    # returns eps_hk_wdt_t struct
    def get_hk_wdt(self):
        self.write(CMD_GET_HK, [0x03])
        array = self.read(SIZE_EPS_HK_WDT_T)
        return c_bytesToStruct(array, "eps_hk_wdt_t")

    # returns eps_hk_basic_t struct
    def get_hk_2_basic(self):
        self.write(CMD_GET_HK, [0x04])
        array = self.read(SIZE_EPS_HK_BASIC_T)
        return c_bytesToStruct(array, "eps_hk_basic_t")

    # sets voltage output channels with bit mask: 
    # byte [1 byte] -> [NC NC 3.3V3 3.3V2 3.3V1 5V3 5V2 5V1]
    # raises: ValueError if byte is not char
    def set_output(self, byte):
        self.write(CMD_SET_OUTPUT, [byte])

    # sets a single output on or off:
    # channel [1 byte]  -> voltage = (0~5), BP4 heater = 6, BP4 switch = 7
    # value   [1 byte]  -> on = 1, off = 0
    # delay   [2 bytes] -> [seconds]
    # raises: AssertionError if channel or value are out of range
    # 		  AssertionError if delay is not a number
    def set_single_output(self, channel, value, delay):
        assert 0 <= channel <= 7, "channel must be in range [0, 7]"
        assert value == 0 or value == 1, "value must be 0 or 1"
        d = toBytes(delay, 2)
        self.write(CMD_SET_SINGLE_OUTPUT, [channel, value]+list(d))

    # Set the voltage on the photo-voltaic inputs V1, V2, V3 in mV. 
    # Takes effect when MODE = 2, See SET_PV_AUTO.
    # volt1~volt3 [2 bytes] -> value in mV
    # raises: AssertionError if voltages are over the max pv voltage
    def set_pv_volt(self, volt1, volt2, volt3):
        assert volt1 <= MAX_PV_VOLTAGE and volt2 <= MAX_PV_VOLTAGE and volt3 <= MAX_PV_VOLTAGE
        v = bytearray(6)
        v[0:2] = toBytes(volt1, 2)
        v[2:4] = toBytes(volt2, 2)
        v[4:] = toBytes(volt3, 2)
        self.write(CMD_SET_PV_VOLT, list(v))

    # Sets the solar cell power tracking mode:
    # mode [1 byte] ->
    # MODE = 0: Hardware default power point
    # MODE = 1: Maximum power point tracking
    # MODE = 2: Fixed software powerpoint, value set with SET_PV_VOLT, default 4V
    # raises: AssertionError if mode is not 0, 1, or 2
    def set_pv_auto(self, mode):
        assert mode in [0, 1, 2]
        self.write(CMD_SET_PV_AUTO, [mode])

    # returns bytearray with heater modes
    # command   [1 byte]  -> 0 = Set heater on/off (toggle?)
    # heater    [1 byte]  -> 0 = BP4, 1 = Onboard, 2 = Both
    # mode      [1 byte]  -> 0 = OFF, 1 = ON
    # return    [2 bytes] -> heater modes
    # raises: AssertionError if variables are not in correct range
    def set_heater(self, command, heater, mode):
        assert command == 0 and heater in [0, 1, 2] and mode in [0, 1]
        self.write(CMD_SET_HEATER, [command, heater, mode])
        return self.read(2)

    def get_heater(self):
        self.write(CMD_SET_HEATER, [])
        return self.read(2)

    # resets the boot counter and WDT counters.
    def reset_counters(self):
        self.write(CMD_RESET_COUNTERS, [0x42])

    # resets (kicks) dedicated WDT.
    def reset_wdt(self):
        self.write(CMD_RESET_WDT, [0x78])

    # Use this command to control the config system.
    # cmd [1 byte] -> cmd = 1: Restore default config
    # raises: AssertionError if command is not 1
    def config_cmd(self, command):
        assert command == 1
        self.write(CMD_CONFIG_CMD, [command])

    # returns eps_config_t structure
    def config_get(self):
        self.write(CMD_CONFIG_GET, [])
        return c_bytesToStruct(self.read(SIZE_EPS_CONFIG_T), "eps_config_t")

    # takes eps_config_t struct and sets configuration
    # raises: AssertionError if struct is not eps_config_t
    def config_set(self, struct):
        assert type(struct) == eps_config_t
        array = struct >>_>> c_structToBytes >>_>> bytesToList
        self.write(CMD_CONFIG_SET, array)

    # I almost want to add something to make sure that the hard_reset function isn't accidentally triggered
    # I feel like requiring a "passcode" to run the function as an argument should be used in order to make sure
    # that the person who is firing this function knows the consequences. Something like:
    # def hard_reset(self, passcode):
    #   assert passcode == "yes", "Are you sure you want to execute this command and understand its consequences?"
    #   self.write(CMD_HARD_RESET, [])

    # Send this command to perform a hard reset of the P31u,
    # including cycling permanent 5V and 3.3V and battery outputs.
    def hard_reset(self):
        self.write(CMD_HARD_RESET, [])

    # Use this command to control the config 2 system.
    # cmd [1 byte] -> cmd=1: Restore default config; cmd=2: Confirm current config
    # raises: AssertionError if command is not a valid value
    def config2_cmd(self, command): 
        assert command in [1, 2]
        self.write(CMD_CONFIG2_CMD, [command]) 

    # Use this command to request the P31 config 2.
    # returns esp_config2_t struct
    def config2_get(self):
        self.write(CMD_CONFIG2_GET, [])
        return c_bytesToStruct(self.read(SIZE_EPS_CONFIG2_T), "eps_config2_t")

    # Use this command to send config 2 to the P31
    # and save it (remember to also confirm it)
    # raises: AssertionError if struct is not eps_config2_t
    def config2_set(self, struct):
        assert type(struct) == eps_config2_t
        array = struct >> _ >> c_structToBytes >> _ >> bytesToList
        self.write(CMD_CONFIG2_SET, array)

    # Higher level functions -------------------------------------------------

    # output must be off before the function is called.
    # pulses output [output] high for some amount of 
    # milliseconds [duration]; called after a delay of 
    # [delay] seconds.
    def pulse(self, output, duration, delay=0):
        time.sleep(delay)
        self.set_single_output(output, 1, 0)
        time.sleep(duration*.001)
        self.set_single_output(output, 0, 0)

    # output must be off before the function is called
    # pulses high for duration amount of milliseconds
    def pulse_pi(self, output, duration, delay=0):
        time.sleep(delay)
        GPIO.output(output, GPIO.HIGH)
        time.sleep(duration*.001)
        GPIO.output(output, GPIO.LOW)

    # switches on if [switch] is true, off otherwise, with a 
    # delay of [delay] seconds.
    def electrolyzer(self, switch, delay=0):
        assert type(switch) == bool
        self.set_single_output(OUT_ELECTROLYZER, int(switch), delay)

    # spikes the solenoid for some number of 
    # milliseconds [spike] and holds at 5v for [hold] 
    # milliseconds with delay of [delay] seconds.
    # output must be off before the function is called
    def solenoid(self, spike, hold, delay=0):
        time.sleep(delay)
        GPIO.output(OUT_PI_SOLENOID_ENABLE, GPIO.HIGH) # Enable voltage boost for solenoid current spike
        self.set_single_output(OUT_SOLENOID, 1, 0)
        time.sleep(.001*spike)
        GPIO.output(OUT_PI_SOLENOID_ENABLE, GPIO.LOW) # Disable voltage boost
        time.sleep(.001*hold)
        # GPIO.output(OUT_PI_SOLENOID_ENABLE, GPIO.HIGH) <-- Why is this line needed????
        self.set_single_output(OUT_SOLENOID, 0, 0)

    # pulses sparkplug for some number of 
    # milliseconds [duration] with delay of [delay] seconds.
    # output must be off before the function is called
    def glowplug(self, duration, delay=0):
        time.sleep(delay)
        self.set_single_output(OUT_GLOWPLUG, 1, 0)
        time.sleep(.001*duration)
        self.set_single_output(OUT_GLOWPLUG, 0, 0)

    # turns burnwire on for [duration] seconds, with a 
    # delay of [delay] seconds.
    def burnwire(self, duration, delay=0):
        time.sleep(delay)
        self.set_single_output(OUT_BURNWIRE_1, 1, 0)
        self.set_single_output(OUT_BURNWIRE_2, 1, 0)
        time.sleep(duration/2)
        self.displayAll()
        time.sleep(duration/2)
        self.set_single_output(OUT_BURNWIRE_1, 0, 0)
        self.set_single_output(OUT_BURNWIRE_2, 0, 0)

    def comms(self, transmit):
        if transmit:
            GPIO.output(OUT_PI_COMMS, GPIO.HIGH)
        else:
            GPIO.output(OUT_PI_COMMS, GPIO.LOW)

    def comms_amplifier(self, on):
        assert type(on) == bool, "Input 'on' must be either True (on) or False (off)"
        self.set_single_output(OUT_COMMS_AMP, int(on), 0)

    def adc(self, t, n, gain=2/3):
        output = []
        for i in range(n):
            val = 5*self._adc.read_adc(0, gain)/26676
            output += [val]
            print(val)
            time.sleep(t)
        return output

    def rtc(self, t, n):
        output = []
        for i in range(n):
            val = self._rtc.read_datetime()
            temp = self._rtc.getTemp()
            output += [(val, temp)]
            print(val)
            print(temp)
            time.sleep(t)
        return output

    def gyro(self, dt):
        x = 0
        y = 0
        z = 0
        while True:
            dxyz = self._gyro.Get_CalOut_Value()
            x += dxyz[0]*dt
            y += dxyz[1]*dt
            z += dxyz[2]*dt
            print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))
            time.sleep(dt)

    def adjust_string(self, string, length):
        new = string
        if len(new) > length:
            new = new[:length]
        elif len(new) < length:
            new += " "*(length-len(new))
        return new

    def display_sensors(self, t=0):
        x, y, z, dt = 0, 0, 0, 0.1
        cycles = 0
        condition = lambda: True if t == 0 else cycles <= t

        while True:
            # gyro info
            header_gyro = "                GYRO                "
            dxyz = self._gyro.Get_CalOut_Value()
            x, y, z = x+dxyz[0]*dt, y+dxyz[1]*dt, z+dxyz[2]*dt
            gyro_r1 = self.adjust_string("x: "+str(x), len(header_gyro))
            gyro_r2 = self.adjust_string("y: "+str(y), len(header_gyro))
            gyro_r3 = self.adjust_string("z: "+str(z), len(header_gyro))

            # adc info
            header_adc = "        PRESSURE        "
            adc_r1 = self.adjust_string(str(300*self._adc.read_adc(0, 2/3)/26676)+" psi", len(header_adc))
            adc_r2 = " "*len(header_adc)
            adc_r3 = adc_r2

            # rtc info
            header_rtc = "                         RTC                         "
            rtc_r1 = self.adjust_string("time: "+str(self._rtc.read_datetime()), len(header_rtc))
            rtc_r2 = self.adjust_string("temp: "+str(self._rtc.getTemp())+" C", len(header_rtc))
            rtc_r3 = " "*len(header_rtc)

            # print everything
            print("|%s|%s|%s|" % (header_gyro,  header_adc, header_rtc))
            print("|%s|%s|%s|" % (gyro_r1,      adc_r1,     rtc_r1))
            print("|%s|%s|%s|" % (gyro_r2,      adc_r2,     rtc_r2))
            print("|%s|%s|%s|" % (gyro_r3,      adc_r3,     rtc_r3))
            print("\033[F"*5)

            time.sleep(dt)

    def nasa_demo(self):
        r = raw_input("electrolyzers: ")
        self.electrolyzer(True)
        time.sleep(float(r))
        self.electrolyzer(False)

        r = raw_input("sparkplug:")
        self.sparkplug(3)

        r = raw_input("solenoid:")
        self.solenoid(10, 250)

        r = raw_input("burnwire:")
        self.burnwire(4)

        r = raw_input("sensor data:")
        self.display_sensors()

    def capture(self):
        os.system("cd ivport-master\npython ivport_capture_A.py")

    def chamber(self, name, t):
        text_file = open(name, "w")
        header = "time:"+str(t)+"\n"
        text_file.write(header)
        while True:
            try:
                hk = self.get_hk_1()
                nxt = "pv "+str(hk.pv[0])+","+str(hk.pv[1])+","+str(hk.pv[2])+"\n" + \
                      "pc "+str(hk.pc)+"\n" + \
                      "bv "+str(hk.bv)+"\n" + \
                      "sc "+str(hk.sc)+"\n" + \
                      "temp "+str(hk.temp[0])+","+str(hk.temp[1])+","+str(hk.temp[2])+","+str(hk.temp[3])+"\n@\n"
                print(nxt)
                text_file.write(nxt)
            except:
                print("recovered from error")
            time.sleep(t)
