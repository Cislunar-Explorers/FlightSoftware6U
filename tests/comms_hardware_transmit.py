from communications.satellite_radio import Radio
from communications.commands import CommandHandler

ch = CommandHandler()
radio = Radio()  # Do not modify
num_sent = 0

proceed = 1
while proceed > 0:
    signal = ch.pack_command(num_sent, 8, 5)
    proceed = int(input("Do you want to send this signal?: " + str(signal.hex())))
    if proceed > 0:
        radio.transmit(signal)
        # radio.transmit(bytearray(signal))
        num_sent += 1
