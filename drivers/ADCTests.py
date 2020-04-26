from ADCDriver import ADC
import time

assert sum([1, 2, 3]) == 6, "Should be 6"

testADC = ADC.init()

while True:
    print(testADC.readPressure())
