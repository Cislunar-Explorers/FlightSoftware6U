from communications.satellite_radio import Radio
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
import time

#Setup
ch = CommandHandler()
groundstation = Radio()

#Invalid counter
#badCounterCommand = ch.pack_command(2,8,7)
#groundstation.transmit(badCounterCommand)

#Invalid MAC
invalidMAC = ch.pack_command(1,8,7)
groundstation.transmit(invalidMAC)

#Shutdown via command
#shutdown = ch.pack_command(1,8,11)
#groundstation.transmit(shutdown)