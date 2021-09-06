import time

from communications.groundstation import Groundstation
from communications.commands import CommandHandler

# Radio setup
gs = Groundstation()
ch = CommandHandler()
# Pack command into a bytearray
ch.register_new_command(1, 2, number1=7, number2=4)
command = ch.pack_command(1, 1, 2, number1=7, number2=4)

# Send the command to the satellite
gs.transmit(command)

print('Transmitted')
print('Entering Receiving Mode')

cycleNumber = 1

# Enter listening mode
while True:

    print('Cycle: ' + str(cycleNumber))

    message = gs.receiveSignal()

    if message is not None:
        print(ch.unpack_command(message))
        print('Terminated')
        break

    else:
        cycleNumber += 1
        time.sleep(1)
