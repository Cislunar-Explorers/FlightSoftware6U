from communications.satellite_radio import Radio
from drivers.gom import Gomspace

radio = Radio()
gom = Gomspace()
transmissionCounter = 0

#Turn off power amplifier
gom.set_PA(on=False)
print('Power amplifier turned off')

#Set RF transmitting side to low
gom.rf_transmitting_switch(receive=True)
print('RF transmitting set to low')

#Turn on LNA
gom.lna(True)
print('LNA turned on')

#Set RF receiving side to high
gom.rf_receiving_switch(receive=True)
print('RF receiving set to high')

while True:
    message = radio.receiveSignal()
    if message is not None:
        print('Transmission #' + transmissionCounter + ' Received: ' + str(message))
        transmissionCounter += 1