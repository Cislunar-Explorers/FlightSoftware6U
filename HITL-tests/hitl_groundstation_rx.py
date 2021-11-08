from communications.satellite_radio import Radio
from communications.command_handler import CommandHandler
groundstation = Radio()
dh = CommandHandler()

print('Receiving...')
while True:
    downlink = groundstation.receiveSignal()
    if downlink is not None:
        print('Downlink Received')
        print(dh.unpack_telemetry(downlink))
