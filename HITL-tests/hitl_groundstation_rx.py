from communications.satellite_radio import Radio
from communications.downlink import DownlinkHandler

groundstation = Radio()
dh = DownlinkHandler()

print('Receiving...')
while True:
    downlink = groundstation.receiveSignal()
    if downlink is not None:
        print('Downlink Received')
        print(dh.unpack_downlink(downlink))