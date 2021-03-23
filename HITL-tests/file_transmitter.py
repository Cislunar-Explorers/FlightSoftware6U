from utils.constants import MIN_COMMAND_SIZE, FMEnum, CommandCommandEnum, FLIGHT_SOFTWARE_PATH
from communications.satellite_radio import Radio
from communications.commands import CommandHandler
import hashlib
import time

groundstation = Radio()
ch = CommandHandler()
command_counter = 1
transmission_interval = 4
file_path_for_opening = FLIGHT_SOFTWARE_PATH + 'FlightSoftware/HITL-tests/test_upload_file.py'
file_path = 'HITL-tests/test_upload_file.py'

#Get file
#Max transmission size - space alotted for file name - block number - 
# min command size - 2*(2 bytes for string length)
max_string_size = 80 - len(file_path) - 2 - MIN_COMMAND_SIZE - 4
file = open(file_path_for_opening)
file_string = file.read()
file_blocks = []

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
    CommandCommandEnum.AddFileBlock.value, file_path = file_path,
    block_number = block[0],block_text = block[1])

    print(block_command)

    groundstation.transmit(block_command)
    print('Transmitted Block #' + str(block[0]))
    time.sleep(transmission_interval)
    command_counter+=1