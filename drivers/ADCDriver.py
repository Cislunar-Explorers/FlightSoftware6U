#
# CislunarExplorers/FlightSoftware/ADCDriver.py
#
# Created by Stefan Brechter (scb262@cornell.edu) on 03/11/2019
#
# Cislunar Explorers
# Space Systems Design Studio
# Cornell University

import ADS1115
import time


class ADC:
    def init(self, ads):
        ads = ADS1115.ADS1115()

    # Read the pressure from the pressure transducer at channel 0 on the ads1115
    def readPressure(self, ads):
        # ads.config = ads.__ADS1015_REG_CONFIG_MUX_SINGLE_0
        milVolts = ads.readADCSingleEnded(channel=0)
        pressure = milVolts / 5 * 300
        return pressure

