import time
import board
import datetime

# For hardware I2C (M0 boards) use this line:
import busio as io

# Or for software I2C (ESP8266) use this line instead:
# import bitbangio as io

import adafruit_ds3231


class TimeTest:
    i2c = io.I2C(board.SCL, board.SDA)  # Change to the appropriate I2C clock & data
    # pins here!

    # Create the RTC instance:
    rtc = adafruit_ds3231.DS3231(i2c)

    def drift_rate(self):
        t = datetime.datetime.now()  # current time= year, mon, date, hour, min, sec and microseconds
        self.rtc.datetime = t    # reset rtc to current time
        print("Set DS3132 time to: ", t)

        # waiting sequence
        time_span = 30  # number of seconds for test
        print("Finding Drift Rate")
        for i in range(time_span):
            print(".")
            time.sleep(1)

        # calculate drift rate
        rtc_time = self.rtc.datetime
        clock_time = datetime.datetime.now()

        drift_time = abs(rtc_time - clock_time)
        drift_rate = drift_time.total_seconds()/time_span
        print("DS3132 Drift Rate: %02f seconds per clock second" % drift_rate)

        # check drift rate
        thresh_acc = 0.0000035   # data sheet informed accuracy 3.5ppm
        try:
            assert drift_rate < thresh_acc
        except AssertionError:
            print("Test Failed: Drift Rate surpasses Data Sheet Threshold Accuracy of 3.5ppm")
        else:
            print("Test Passed - Drift Rate falls within Data Sheet Threshold Accuracy of 3.5ppm")

