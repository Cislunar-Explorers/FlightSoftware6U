from fsw.communications.groundstation import Groundstation
from fsw.communications.commands import CommandHandler
from fsw.communications.downlink import DownlinkHandler
from fsw.flight_modes.flight_mode_factory import FLIGHT_MODE_DICT
import time
import logging


ch = CommandHandler()
dh = DownlinkHandler()
groundstation = Groundstation()
commandCounter = 1
transmitInterval = 3
rx_wait_time = 20  # seconds

while True:
    commandInput = input(
        'Enter command in the form "Mode ID,Command ID, Keyword Arguments" (e.g 2,3,delay=3,state=True):'
    )

    commandArguments = commandInput.split(",")
    mode_id = int(commandArguments[0])
    command_id = int(commandArguments[1])
    kwargs = {}

    if len(commandArguments) > 2:
        argsList = commandArguments[2:]
        for i in range(len(argsList)):

            signIndex = argsList[i].index("=")
            argName = argsList[i][:signIndex]
            argValue = argsList[i][signIndex + 1:]

            arg_type = FLIGHT_MODE_DICT[mode_id].command_arg_types[argName]
            if arg_type == "int" or arg_type == "short":
                argValue = int(argValue)
            elif arg_type == "float" or arg_type == "double":
                argValue = float(argValue)
            elif arg_type == "bool":
                if argValue == "True":
                    argValue = True
                else:
                    argValue = False

            kwargs[argName] = argValue

    commandToTransmit = ch.pack_command(commandCounter, mode_id, command_id, **kwargs)
    logging.info(len(commandToTransmit))
    logging.info(commandToTransmit.hex())
    groundstation.transmit(commandToTransmit)
    commandCounter += 1
    logging.info(
        "Successfully transmitted " + str(ch.unpack_command(commandToTransmit))
    )
    time.sleep(transmitInterval)

    logging.info("Receiving...")

    rx_start_time = time.time()
    while (time.time() - rx_start_time) < rx_wait_time:
        downlink = groundstation.receiveSignal()
        if downlink is not None:
            logging.info("Downlink Received")
            logging.info(dh.unpack_downlink(downlink))
            break
