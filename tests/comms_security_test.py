from communications.satellite_radio import Radio
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
import time

#Setup
ch = CommandHandler()
groundstation = Radio()

#Invalid counter
badCounterCommand = ch.pack_command(1,8,7)
groundstation.transmit(badCounterCommand)