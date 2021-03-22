from communications.satellite_radio import Radio
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
import time

#Setup
ch = CommandHandler()
dh = DownlinkHandler()
groundstation = Radio()

#Send command to get gyro/mag/acc data
gmaCommand = ch.pack_command(1,8,7)
groundstation.transmit(gmaCommand)
print('GMA Command Transmitted')

#Listen for response
print('Receiving...')
cycle = 1

while True:
    print('Receiving Cycle: ' + str(cycle))
    
    downlink = groundstation.receiveSignal()
    
    if downlink is not None:
        print('Downlink Received:')
        data= dh.unpack_downlink(downlink)[2]
        print('Gyro: ' + str(data['gyro1']) + ', ' + str(data['gyro2']) + ', ' + str(data['gyro3']))
        break

    cycle+=1
    time.sleep(1)
time.sleep(5)
#Bogus command testing
bogusCommand = b'\x09\x14\x03\x00'
groundstation.transmit(bogusCommand)
print('Bogus Command Transmitted')