from communications.satellite_radio import Radio
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler

#Setup
ch = CommandHandler()
dh = DownlinkHandler()
groundstation = Radio()

#Send command to get gyro/mag/acc data
gmaCommand = ch.pack_command(8,7)
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
        print('Gyro: ' + str(data['gyro']) + ', Mag: ' + str(data['mag']) + ', Acc: ' + str(data['acc']))
        break

#Bogus command testing
ch.register_new_command(0,3,alex=4)
bogusCommand = ch.pack_command(0,3,alex=4)
groundstation.transmit(bogusCommand)
print('Bogus Command Transmitted')

#Listen for response
print('Receiving...')
cycle = 1

while True:
    print('Receiving Cycle: ' + str(cycle))
    
    downlink = groundstation.receiveSignal()
    
    if downlink is not None:
        print('Downlink Received:')
        data= dh.unpack_downlink(downlink)[2]
        print(data)
        break
