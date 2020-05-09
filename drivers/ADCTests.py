from ADCDriver import ADC
import time

assert sum([1, 2, 3]) == 6, "Should be 6"


def test_ADC_initialize():
    return ADC


def test_ADC_read_pressure(testADC):
    while True:
        print(testADC.readPressure(testADC))
        time.sleep(1)


def test_ADC_read_tempVolt(testADC):
    while True:
        print(testADC.readTemperature(testADC))
        time.sleep(1)


if True:
    testADC = test_ADC_initialize()
    # test_ADC_read_pressure(testADC)
    test_ADC_read_tempVolt(testADC)
