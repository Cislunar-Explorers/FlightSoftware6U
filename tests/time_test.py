from utils.log import *
import time
import board
import datetime

# For hardware I2C (M0 boards) use this line:
import busio as io

# Or for software I2C (ESP8266) use this line instead:
# import bitbangio as io

import adafruit_ds3231


class RTC:

    def __init__(self):
        self.i2c = io.I2C(board.SCL, board.SDA)  # Change to appropriate I2C clock & data pins

        # Create RTC instance:
        self.rtc = adafruit_ds3231.DS3231(self.i2c)

    def reset_rtc(self):
        t = datetime.datetime.now()  # current time= year, mon, date, hour, min, sec and microseconds
        self.rtc.datetime = t  # reset rtc to current time
        log.info('Set DS3132 time to: {}'.format(t))

    def drift_rate(self, time_span):
        # time span
        log.info('Time span: {}'.format(time_span))

        # set current time
        self.reset_rtc()
        clock_time_init = datetime.datetime.now()

        # waiting sequence
        print('Finding Drift Rate')
        for i in range(time_span):
            if i % 10 == 0:
                print('{} seconds passed'.format(i))
            time.sleep(1)

        # calculate drift rate
        rtc_time_final = self.rtc.datetime
        clock_time_final = datetime.datetime.now()
        d_time = abs(rtc_time_final - clock_time_final)
        d_rate = d_time.total_seconds() / abs(clock_time_final - clock_time_init)

        log.info('Drift Time: {} seconds'.format(d_time))
        log.info('DS3231 Drift Rate: {} seconds per clock second'.format(d_rate))

        # check accuracy
        thresh_acc = 0.0000035  # data sheet informed accuracy 3.5ppm
        try:
            assert d_rate < thresh_acc
        except AssertionError:
            log.info('Drift Rate NOT within Data Sheet Threshold Accuracy of 3.5ppm')
        else:
            log.info('Drift Rate within Data Sheet Threshold Accuracy of 3.5ppm')

    def recover(self):
        self.reset_rtc()    # reset RTC


rtc1 = RTC()
rtc1.drift_rate(5)
rtc1.drift_rate(20)
rtc1.drift_rate(300)

