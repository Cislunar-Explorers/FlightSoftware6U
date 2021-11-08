from utils.constants import DATA_OFFSET, ID_SIZE, MAC_LENGTH, COUNTER_OFFSET, COUNTER_SIZE, MAC_KEY, ID_OFFSET, DATA_LEN_OFFSET, MIN_COMMAND_SIZE

from typing import List, Tuple, Dict
import hashlib
from communications.commands import Command
from communications.command_definitions import COMMAND_LIST
from utils.exceptions import CommandUnpackingException
import logging


def compute_mac(data: bytes) -> bytes:
    """Computes the message authentication code for what's input into `data`"""
    return hashlib.blake2s(data, digest_size=MAC_LENGTH, key=MAC_KEY).digest()


def verify_mac(data: bytes) -> bool:
    """Verifies that the data received has the correct MAC. Returns true if the MACs are the same"""
    # get the mac from the uplink
    received_mac = data[:MAC_LENGTH]

    # compute the mac that the message should have
    computed_mac = compute_mac(data[MAC_LENGTH:])

    return received_mac == computed_mac


class CommandHandler:
    """Handling the packing, unpacking, and execution of uplinks (commands) and downlinks (telemetry)"""
    command_list: List[Command] = COMMAND_LIST
    uplink_counter: int
    downlink_counter: int

    def get_command_from_id(self, command_id: int):
        try:
            return [command for command in self.command_list if command.id == command_id][0]
        except IndexError:
            raise CommandUnpackingException(f"Command {command_id} not found")

    def unpack_link(self, data: bytes, uplink: bool = True) -> Tuple[Command, Dict]:
        if verify_mac(data):
            # TODO deal with counter
            counter = int.from_bytes(
                data[COUNTER_OFFSET:COUNTER_OFFSET + COUNTER_SIZE], 'big')

            mac = data[:MAC_LENGTH]
            command_id = data[ID_OFFSET]
            data_length = data[DATA_LEN_OFFSET]
            kwarg_data = data[DATA_OFFSET:]

            if data_length != len(kwarg_data):
                raise CommandUnpackingException(
                    f"Data Length and command data discrepancy. Data's actual length is {len(kwarg_data)}, but was said to be {data_length}")

            # get command
            command = self.get_command_from_id(command_id)

            if uplink:
                kwargs = command.unpack_args(kwarg_data)
            else:
                kwargs = command.unpack_telem(kwarg_data)

            logging.info(
                f"Received command: MAC: {mac.hex()}, ID: {command_id}, Counter: {counter}, Args: {kwargs} ")

            return command, kwargs
        else:
            logging.warning(f"Incorrect MAC, ignoring command {data.hex()}")
            raise CommandUnpackingException(
                "MAC discrepancy - not running command")

    def pack_link(self, uplink: bool, counter: int, command_id: int, kwargs) -> bytes:
        command = self.get_command_from_id(command_id)

        if uplink:
            buffer_size = command.uplink_buffer_size
            link_data = command.pack_args(kwargs)
        else:
            buffer_size = command.downlink_buffer_size
            link_data = command.pack_telem(kwargs)

        data_buffer = bytearray(
            MIN_COMMAND_SIZE - MAC_LENGTH + buffer_size)
        data_buffer[:COUNTER_SIZE] = counter.to_bytes(COUNTER_SIZE, 'big')
        data_buffer[ID_OFFSET - MAC_LENGTH] = command_id
        data_buffer[DATA_LEN_OFFSET - MAC_LENGTH] = len(link_data)
        data_buffer[DATA_OFFSET - MAC_LENGTH:] = link_data

        mac = compute_mac(bytes(data_buffer))
        return mac + bytes(data_buffer)