from fsw.communications.satellite_radio import Radio
from fsw.communications.command_handler import CommandHandler
groundstation = Radio()
dh = CommandHandler(None)

print('Receiving...')
while True:
    downlink = groundstation.receiveSignal()
    if downlink is not None:
        print('Downlink Received')
        print(dh.unpack_telemetry(downlink))
