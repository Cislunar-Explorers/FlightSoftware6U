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
    ads = ADS1115.ADS1115()

    def __init__(self):
        self.ads = ADS1115.ADS1115()

    # Read the pressure from the pressure transducer at channel 0 on the ads1115
    def readPressure(self):
        milVolts = self.ads.readADCSingleEnded(channel=0)
        pressure = round(milVolts / 5000 * 300, 3)
        return pressure

        # Read the voltage difference between pins

    def tempVoltage(self):
        pos = self.ads.readADCSingleEnded(channel=3)
        neg = self.ads.readADCSingleEnded(channel=2)
        return pos - neg

    def readTemperature(self):
        pos = self.ads.readADCSingleEnded(channel=3)
        neg = self.ads.readADCSingleEnded(channel=2)
        hotJuncVolts = pos - neg
        return hotJuncVolts

        # coldJuncVolts = coldVoltage(self, ads)

    # def coldVoltage(self, ads):
    # TODO

    # def convertVoltToTemp(self, ads):

    # def convertTempToVolt(self, ads):
