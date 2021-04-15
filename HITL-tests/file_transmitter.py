from utils.constants import MIN_COMMAND_SIZE, FMEnum, CommandCommandEnum, FLIGHT_SOFTWARE_PATH
from communications.satellite_radio import Radio
from communications.commands import CommandHandler
from communications.downlink import DownlinkHandler
import hashlib
import time

groundstation = Radio()
ch = CommandHandler()
command_counter = 1
transmission_interval = 4
file_path= FLIGHT_SOFTWARE_PATH + 'FlightSoftware/HITL-tests/test_file'

#Get file
#Max transmission size - space alotted for file name - block number - 
# min command size - 2*(2 bytes for string length)
max_string_size = 195 - MIN_COMMAND_SIZE
file = open(file_path)
file_string = file.read()
file_blocks = []
print('File Checksum: ' + str(hashlib.md5(file_string.encode('utf-8')).hexdigest()))

#Determine number of blocks
number_of_blocks = len(file_string)//max_string_size
if len(file_string) % max_string_size != 0:
    number_of_blocks += 1

for i in range(number_of_blocks):
    block_text = file_string[i*max_string_size:(i+1)*max_string_size]
    file_blocks.append((i,block_text))

i = 0
#Transmit blocks
for block in file_blocks:
    
    block_command = ch.pack_command(command_counter, FMEnum.Command.value, 
    CommandCommandEnum.AddFileBlock.value,
    block_number = block[0],block_text = block[1])

    groundstation.transmit(block_command)
    print('Transmitted Block #' + str(block[0]))
    time.sleep(transmission_interval)
    command_counter+= 1

file_info_request = ch.pack_command(command_counter, FMEnum.Command.value,
CommandCommandEnum.GetFileBlocksInfo.value,total_blocks=number_of_blocks)
groundstation.transmit(file_info_request)
print('Receiving...')
while True:
    downlink = groundstation.receiveSignal()
    if downlink is not None:
        print('Downlink Received')
        print(dh.unpack_downlink(downlink))