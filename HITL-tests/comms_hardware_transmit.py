from fsw.communications .satellite_radio import Radio
import time

# Test constants
radio = Radio()  # Do not modify
signal = b'\x08\x07\x00\x00'  # Pattern you want to send
interval = 3  # Number of seconds between each transmission
repetitions = 10  # Number of total times you want to transmit the pattern

for i in range(1, repetitions+1):
    radio.transmit(signal)
    print('Transmitted: ' + str(i))
    time.sleep(interval)
