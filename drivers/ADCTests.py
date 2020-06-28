# CislunarExplorers/FlightSoftware/drivers/ADCTests.py
#
# Created by Stefan Brechter (scb262@cornell.edu) on 03/11/2020
#
# Cislunar Explorers
# Space Systems Design Studio
# Cornell University
#
# For detailed descriptions and analysis of the following test file please
# look at the accompanying End of Semester Report found here:
# https://cornell.app.box.com/file/664230352636

from ADCDriver import ADC
import time

assert sum([1, 2, 3]) == 6, "Should be 6"


def test_ADC_initialize():
    return ADC


def test_ADC_read_pressure():
    print(ADC.read_pressure())


def test_ADC_read_pressure_continuous():
    while True:
        print(ADC.read_pressure())
        time.sleep(1)


def test_ADC_read_pressure_20():
    x = 0
    print("These are pressure readings when the analog to digital converter is connected to the 0 volt ground pin, 3.3V, and 5V on the RPi.")
    print("Ouput given in psi.")
    print("Readings taken once per second for 20 seconds.")
    while x < 20:
        test_ADC_read_pressure()
        x = x + 1
        time.sleep(1)


def test_ADC_read_temperature():
    print("--------------------------")
    print("Celsius:")
    cel = ADC.read_temperature()
    print(cel)
    print("Fahrenheit:")
    print(cel * 1.8 + 32)


def test_ADC_read_temperature_continuous():
    while True:
        test_ADC_read_temperature()
        time.sleep(1)


def test_ADC_read_temperature_20():
    x = 0
    while x < 20:
        test_ADC_read_temperature()
        x = x + 1
        time.sleep(1)


def test_ADC_get_gyro_temp():
    print("Cold junction temperature fro gyro sensor in Celsius:")
    print(ADC.get_gyro_temp())


if True:
    testADC = test_ADC_initialize()
    test_ADC_get_gyro_temp(testADC)
    # test_ADC_read_pressure_20(testADC)
    print("Conversion sanity check: 25.6 degrees")
    print(ADC.convert_volt_to_temp(ADC.convert_temp_to_volt(25.6)))
    print("Conversion sanity check: 2.023 mV")
    print(ADC.convert_temp_to_volt(ADC.convert_volt_to_temp(2.023)))
    test_ADC_read_temperature_continuous()
