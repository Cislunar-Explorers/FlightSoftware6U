import time
from math import ceil, floor
from fractions import Fraction
from picamera import PiCamera, mmal, PiVideoFrameType
from picamera.mmalobj import to_rational
import os

# Analog and digital gain parameters (not exposed in picamera-1.13)
MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A

# Experiment with mode 4 (half-resolution) to reduce severity of rolling shutter
with PiCamera(resolution = (3280//2, 2464//2),
              framerate = 15,
              sensor_mode = 4,
              clock_mode = 'raw') as camera:
    # Set fixed white balance
    # (gain choices from absolute radiometric calibration by Pagnutti et al.)
    camera.awb_mode = 'off'
    camera.awb_gains = (Fraction(525, 325), Fraction(425, 325))

    # Set fixed analog and digital gains
    # Max analog gain: Fraction(2521,256)
    mmal.mmal_port_parameter_set_rational(camera._camera.control._port, MMAL_PARAMETER_ANALOG_GAIN, to_rational(1))
    mmal.mmal_port_parameter_set_rational(camera._camera.control._port, MMAL_PARAMETER_DIGITAL_GAIN, to_rational(1))

    # Set shutter speed [us]
    camera.shutter_speed = 9

    # Wait for parameter changes to take effect, then lock gains
    # (locking gains may not be required)
    time.sleep(1)
    camera.exposure_mode = 'off'

    t0 = time.monotonic()
    camera.start_recording('pivideo.mjpg', format='mjpeg')
    lastTimestamp = camera.timestamp
    lastIndex = -1
    for n in range(15):
        f = camera.frame
        while f is None or f.index == lastIndex or f.frame_type == PiVideoFrameType.sps_header:
            f = camera.frame
            time.sleep((1/15)/2)
        dt = f.timestamp - lastTimestamp
        print('%6d\t%s' % (dt, f))
        lastIndex = f.index
        lastTimestamp = f.timestamp

    camera.stop_recording()
    print('Generating frames')
    os.system('ffmpeg -i pivideo.mjpg -vcodec copy frame%02d.jpg')
