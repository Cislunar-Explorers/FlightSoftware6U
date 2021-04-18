from communications.satellite_radio import Radio
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
from flight_modes.flight_mode_factory import FLIGHT_MODE_DICT
import time

ch = CommandHandler()
groundstation = Radio()
commandCounter = 1
transmitInterval = 3

commandInput = input('Enter command in the form "Mode ID,Command ID, Keyword Arguments" (e.g 2,3,delay=3,state=True):')

commandArguments = commandInput.split(',')
mode_id = int(commandArguments[0])
command_id = int(commandArguments[1])
kwargs = {}

if len(commandArguments) > 2:
    argsList = commandArguments[2:]
    for i in range(len(argsList)):
        
        signIndex = argsList[i].index('=')
        argName = argsList[i][:signIndex]
        argValue = argsList[i][signIndex +1:]

        arg_type = FLIGHT_MODE_DICT[mode_id].command_arg_types[argName]
        if arg_type == 'int' or arg_type == 'short':
            argValue = int(argValue)
        elif arg_type == 'float' or arg_type == 'double':
            argValue = float(argValue)
        elif arg_type == 'bool':
            if argValue == 'True':
                argValue = True
            else:
                argValue = False
        
        kwargs[argName] = argValue

commandToTransmit = ch.pack_command(commandCounter, mode_id, command_id, **kwargs)
print(len(commandToTransmit))
print(commandToTransmit)
groundstation.transmit(commandToTransmit)
commandCounter += 1
print('Successfully transmitted ' + str(ch.unpack_command(commandToTransmit)))
time.sleep(transmitInterval)

dh = DownlinkHandler()

print('Receiving...')
while True:
    downlink = groundstation.receiveSignal()
    if downlink is not None:
        print('Downlink Received')
        print(dh.unpack_downlink(downlink))