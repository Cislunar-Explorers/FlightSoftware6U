# Camera mux imports
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_bus_device.i2c_device import I2CDevice
from vcgencmd import Vcgencmd

# Video capture imports
import time
from math import ceil, floor
from fractions import Fraction
from picamera import PiCamera, mmal, PiVideoFrameType
from picamera.mmalobj import to_rational


class CameraMux:
    def __init__(self):
        # Camera mux hardware definitions
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.mux = I2CDevice(self.i2c, 0x70)

        self.sel_pin = DigitalInOut(board.D4)
        self.sel_pin.direction = Direction.OUTPUT
        self.oe1_pin = DigitalInOut(board.D17)
        self.oe1_pin.direction = Direction.OUTPUT
        self.oe2_pin = DigitalInOut(board.D18)
        self.oe2_pin.direction = Direction.OUTPUT

    def selectCamera(self, id):
        if id == 1:
            with self.mux:
                self.mux.write(bytes([0x04]))
                self.sel_pin.value = False
                self.oe1_pin.value = False
                self.oe2_pin.value = True
        elif id == 2:
            with self.mux:
                self.mux.write(bytes([0x05]))
                self.sel_pin.value = True
                self.oe1_pin.value = False
                self.oe2_pin.value = True
        elif id == 3:
            with self.mux:
                self.mux.write(bytes([0x06]))
                self.sel_pin.value = False
                self.oe1_pin.value = True
                self.oe2_pin.value = False
        elif id == 4:
            with self.mux:
                self.mux.write(bytes([0x07]))
                self.sel_pin.value = True
                self.oe1_pin.value = True
                self.oe2_pin.value = False
        else:
            assert False

    def detect(self):
        vcgm = Vcgencmd()
        status = vcgm.get_camera()
        detected = status['detected']
        if detected == 1:
            return True
        else:
            return False


class Camera:
    def __init__(self):
        pass

    def initialize(self):
        return PiCamera(resolution=(3280, 2464),
                      framerate=15,
                      sensor_mode=2,
                      clock_mode='raw')


    # Code from video-timing.py
    def rawObservation(filename, frame_rate=15, shutterSpeed = 30000):

        # Analog and digital gain parameters (not exposed in picamera-1.13)
        MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
        MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A

        # Full resolution
        with PiCamera(resolution=(3280, 2464),
                      framerate=15,
                      sensor_mode=2,
                      clock_mode='raw') as camera:
            # Set fixed white balance
            # (gain choices from absolute radiometric calibration by Pagnutti et al.)
            camera.awb_mode = 'off'
            camera.awb_gains = (Fraction(525, 325), Fraction(425, 325))

            # Set fixed analog and digital gains
            # Max analog gain: Fraction(2521,256)
            # Note: will want minimum gain (1?) for Sun, maximum analog gain for Earth/Moon
            mmal.mmal_port_parameter_set_rational(camera._camera.control._port, MMAL_PARAMETER_ANALOG_GAIN,
                                                  to_rational(Fraction(2521, 256)))
            mmal.mmal_port_parameter_set_rational(camera._camera.control._port, MMAL_PARAMETER_DIGITAL_GAIN, to_rational(1))

            # Set shutter speed [us]
            # Note: will want minimum shutter speed (19us) for Sun, slower for Earth/Moon
            camera.shutter_speed = shutterSpeed

            # Wait for parameter changes to take effect, then lock gains
            # (locking gains may not be required)
            time.sleep(1)
            camera.exposure_mode = 'off'

            # t0 = time.monotonic()  # Host time when recording is commanded
            camera.start_recording(filename, format='mjpeg')
            lastTimestamp = camera.timestamp
            lastIndex = -1
            # Loop over expected frames
            for n in range(2 * frame_rate):
                f = camera.frame
                # If no new frame yet, sleep for half a frame
                while f is None or f.index == lastIndex or f.frame_type == PiVideoFrameType.sps_header:
                    f = camera.frame
                    time.sleep((1 / frame_rate) / 2)
                lastIndex = f.index
                lastTimestamp = f.timestamp
            camera.stop_recording()
            return {filename: lastTimestamp}
