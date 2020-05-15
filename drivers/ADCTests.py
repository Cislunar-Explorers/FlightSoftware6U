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


def test_ADC_get_gyro_temp(testADC):
    print("Cold junction temperature fro gyro sensor in Celsius:")
    print(testADC.get_gyro_temp(testADC))


if True:
    testADC = test_ADC_initialize()
    test_ADC_get_gyro_temp(testADC)
    # test_ADC_read_pressure_20(testADC)
    print("Conversion sanity check: 25.6 degrees")
    print(ADC.convert_volt_to_temp(testADC, ADC.convert_temp_to_volt(testADC, 25.6)))
    print("Conversion sanity check: 2.023 mV")
    print(ADC.convert_temp_to_volt(testADC, ADC.convert_volt_to_temp(testADC, 2.023)))
    test_ADC_read_temperature_continuous(testADC)
