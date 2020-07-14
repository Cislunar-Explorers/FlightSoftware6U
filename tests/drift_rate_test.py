from FlightSoftware.utils.log import *
import time
import board
import datetime

# For hardware I2C (M0 boards) use this line:
import busio as io

# Or for software I2C (ESP8266) use this line instead:
# import bitbangio as io

import adafruit_ds3231


class TimeTest:
    i2c = io.I2C(board.SCL, board.SDA)  # Change to appropriate I2C clock & data pins

    # Create RTC instance:
    rtc = adafruit_ds3231.DS3231(i2c)

    def __init__(self):
        pass

    def drift_rate(self, time_span):

        # time span
        log.info('Time span: {}'.format(time_span))

        # set current time
        t = datetime.datetime.now()  # current time= year, mon, date, hour, min, sec and microseconds
        self.rtc.datetime = t    # reset rtc to current time
        log.info('Set DS3132 time to: {}'.format(t))

        # waiting sequence
        print('Finding Drift Rate')
        for i in range(time_span):
            print('.')
            time.sleep(1)

        # calculate drift rate
        rtc_time = self.rtc.datetime
        clock_time = datetime.datetime.now()
        drift_time = abs(rtc_time - clock_time)
        drift_rate = drift_time.total_seconds()/time_span

        log.info('Drift Time: {} seconds'.format(drift_time))
        log.info('DS3231 Drift Rate: {} seconds per clock second'.format(drift_rate))

        # check accuracy
        thresh_acc = 0.0000035   # data sheet informed accuracy 3.5ppm
        try:
            assert drift_rate < thresh_acc
        except AssertionError:
            log.info('Drift Rate NOT within Data Sheet Threshold Accuracy of 3.5ppm')
        else:
            log.info('Drift Rate within Data Sheet Threshold Accuracy of 3.5ppm')
