from __future__ import annotations
from typing import TYPE_CHECKING, Any, Union
from fsw_utils import parameter_utils

if TYPE_CHECKING:
    from fsw_main.main import MainSatelliteThread

from fsw_utils.constants import (
    DATA_OFFSET,
    MAC_LENGTH,
    COUNTER_OFFSET,
    COUNTER_SIZE,
    MAC_KEY,
    ID_OFFSET,
    DATA_LEN_OFFSET,
    MIN_COMMAND_SIZE,
    ONE_WORD,
    ZERO_WORD,
)

from typing import Optional, Tuple, Dict
import hashlib
from communications.commands import Command
from communications.command_definitions import COMMAND_DICT
from fsw_utils.exceptions import CommandUnpackingException
from communications.downlink import bit_inflation
from communications.groundstation import bit_deflation
import logging
import fsw_utils.parameters as params


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

    uplink_counter: int  # how many times we've received a valid data packet
    downlink_counter: int  # how many times we've downlinked something
    # flag to do bit inflation; must be True for flight (if not done in Radio), can be False for testing
    inflation: bool
    _main: Optional[MainSatelliteThread] = None

    def __init__(self, main: Optional[MainSatelliteThread], inflation=False) -> None:
        self.inflation = inflation
        self._main = main
        self.uplink_counter = params.UPLINK_COUNTER
        self.downlink_counter = params.DOWNLINK_COUNTER

        logging.info(
            f"Uplink/Downlink counters: {self.uplink_counter}/{self.downlink_counter}"
        )

    def increment_downlink_counter(self):
        self.downlink_counter += 1
        parameter_utils.set_parameter("DOWNLINK_COUNTER", self.downlink_counter, True)

    def increment_uplink_counter(self):
        self.uplink_counter += 1
        parameter_utils.set_parameter("UPLINK_COUNTER", self.uplink_counter, True)

    def get_command_from_id(self, command_id: int):
        try:
            return COMMAND_DICT[command_id]
        except IndexError:
            raise CommandUnpackingException(f"Command {command_id} not found")

    def unpack_link(
        self, data: bytes, uplink: bool = True
    ) -> Tuple[Command, Dict[str, Any]]:
        if not uplink and self.inflation:
            # deflate bits
            data = bit_deflation(bytearray(data), ZERO_WORD, ONE_WORD)
        if verify_mac(data):
            # TODO deal with counter
            counter = int.from_bytes(
                data[COUNTER_OFFSET : COUNTER_OFFSET + COUNTER_SIZE], "big"
            )

            mac = data[:MAC_LENGTH]
            command_id = data[ID_OFFSET]
            data_length = data[DATA_LEN_OFFSET]
            kwarg_data = data[DATA_OFFSET:]

            if counter < self.uplink_counter and uplink:
                raise CommandUnpackingException(
                    f"Command counter ({counter}) is less than uplink counter ({self.uplink_counter})."
                    f"Ignoring command"
                )

            if counter < self.downlink_counter and not uplink:
                raise CommandUnpackingException(
                    f"Command counter ({counter}) is less than downlink counter ({self.downlink_counter})."
                    f"Ignoring command"
                )

            if data_length != len(kwarg_data):
                raise CommandUnpackingException(
                    f"Data Length and command data discrepancy. Data's actual length is {len(kwarg_data)},"
                    f"but was said to be {data_length}"
                )

            # get command
            command = self.get_command_from_id(command_id)

            if uplink:
                kwargs = command.unpack_args(kwarg_data)
            else:
                kwargs = command.unpack_telem(kwarg_data)

            logging.info(
                f"Received command: MAC: {mac.hex()}, ID: {command_id}, Counter: {counter}, Args: {kwargs} "
            )

            return command, kwargs
        else:
            logging.warning(f"Incorrect MAC, ignoring command {data.hex()}")
            raise CommandUnpackingException("MAC discrepancy - not running command")

    def pack_link(
        self, uplink: bool, counter: int, command_id: int, kwarg_dict: Dict[str, Any]
    ) -> bytes:
        command = self.get_command_from_id(command_id)

        if uplink:
            buffer_size = command.uplink_buffer_size
            link_data = command.pack_args(kwarg_dict)
        else:
            buffer_size = command.downlink_buffer_size
            link_data = command.pack_telem(kwarg_dict)

        data_buffer = bytearray(MIN_COMMAND_SIZE - MAC_LENGTH + buffer_size)
        data_buffer[:COUNTER_SIZE] = counter.to_bytes(COUNTER_SIZE, "big")
        data_buffer[ID_OFFSET - MAC_LENGTH] = command_id
        data_buffer[DATA_LEN_OFFSET - MAC_LENGTH] = len(link_data)
        data_buffer[DATA_OFFSET - MAC_LENGTH :] = link_data

        mac = compute_mac(bytes(data_buffer))
        packet = mac + bytes(data_buffer)

        if not uplink and self.inflation:
            # inflate bits
            packet = bytes(bit_inflation(bytearray(packet), ZERO_WORD, ONE_WORD))

        return packet

    def execute_command(self, data: bytes) -> Optional[bytes]:
        # unpack
        try:
            command, kwargs = self.unpack_link(data)
        except CommandUnpackingException as cue:
            logging.error(cue, exc_info=True)
            logging.warning(f"Ignoring command: {data.hex()}")
            return None
        else:
            # increment counter
            self.increment_uplink_counter()

        # run command
        downlink_data = self.run_command(command, kwargs)
        # pack downlinks
        if downlink_data is not None:
            downlink = self.pack_telemetry(command.id, downlink_data)
        else:
            return None
        return downlink

    def run_command(self, command: Command, kwargs: Dict[str, Union[int, float, str]]):
        return command.run(main=self._main, **kwargs)

    def pack_telemetry(self, id, kwargs) -> bytes:
        telemetry_bytes = self.pack_link(False, self.downlink_counter, id, kwargs)
        self.increment_downlink_counter()
        return telemetry_bytes

    def unpack_telemetry(self, data: bytes):
        unpacked_data = self.unpack_link(data, uplink=False)
        self.downlink_counter += 1
        return unpacked_data

    def pack_command(self, id, kwargs) -> bytes:
        command_bytes = self.pack_link(True, self.uplink_counter, id, kwargs)
        self.uplink_counter += 1
        return command_bytes
