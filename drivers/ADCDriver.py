#!/usr/bin/python
#
# CislunarExplorers/FlightSoftware/drivers/ADCDriver.py
#
# Created by Stefan Brechter (scb262@cornell.edu) on 03/11/2019
#
# Cislunar Explorers
# Space Systems Design Studio
# Cornell University

import ADS1115
import time


class ADC:
    def __init__(self):
        self.ads = ADS1115.ADS1115()

    # Read the pressure from the pressure transducer at channel 0 on the ads1115
    def readPressure(self):
        # ads.config = ads.__ADS1015_REG_CONFIG_MUX_SINGLE_0
        milVolts = ADS1115.ADS1115().readADCSingleEnded(channel=0)
        pressure = milVolts / 5 * 300
        return pressure

    # def readTemperature(self, ads):
    #    hotJuncVolts = tempVoltage(self, ads)
    #    coldJuncVolts = coldVoltage(self, ads)

    # Read the voltage difference between pins
    # def tempVoltage(self, ads):
    #    pos = ads.readADCSingleEnded(channel=3)
    #    neg = ads.readADCSingleEnded(channel=2)

    #    return pos - neg

    # def coldVoltage(self, ads):
    # TODO

    # def convertVoltToTemp(self, ads):

    # def convertTempToVolt(self, ads):
