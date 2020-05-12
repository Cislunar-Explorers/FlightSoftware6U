from ADCDriver import ADC
import time

assert sum([1, 2, 3]) == 6, "Should be 6"


def test_ADC_initialize():
    return ADC


def test_ADC_read_pressure(testADC):
    while True:
        print(testADC.read_pressure(testADC))
        time.sleep(1)


def test_ADC_read_temperature(testADC):
    while True:
        print("--------------------------")
        cel = testADC.read_temperature(testADC)
        print(cel)
        print("Fahrenheit:")
        print(cel * 1.8 + 32)
        time.sleep(1)


if True:
    testADC = test_ADC_initialize()
    # test_ADC_read_pressure(testADC)
    test_ADC_read_temperature(testADC)
