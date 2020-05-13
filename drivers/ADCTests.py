from ADCDriver import ADC
import time

assert sum([1, 2, 3]) == 6, "Should be 6"


def test_ADC_initialize():
    return ADC


def test_ADC_read_pressure(testADC):
    print(testADC.read_pressure(testADC))


def test_ADC_read_pressure_continuous(testADC):
    while True:
        print(testADC.read_pressure(testADC))
        time.sleep(1)


def test_ADC_read_pressure_20(testADC):
    x = 0
    while x < 20:
        test_ADC_read_pressure(testADC)
        x = x + 1
        time.sleep(1)


def test_ADC_read_temperature(testADC):
    print("--------------------------")
    print("Celsius:")
    cel = testADC.read_temperature(testADC)
    print(cel)
    print("Fahrenheit:")
    print(cel * 1.8 + 32)


def test_ADC_read_temperature_continuous(testADC):
    while True:
        test_ADC_read_temperature(testADC)
        time.sleep(1)


def test_ADC_read_temperature_20(testADC):
    x = 0
    while x < 20:
        test_ADC_read_temperature(testADC)
        x = x + 1
        time.sleep(1)


if True:
    testADC = test_ADC_initialize()
    test_ADC_read_pressure_20(testADC)
    # test_ADC_read_temperature(testADC)
