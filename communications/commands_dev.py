from abc import ABC, abstractmethod
from utils.exceptions import CommandUnpackingException
from utils.struct import packer_dict
from typing import Union, Dict, List, Any, Optional, Tuple
from communications.datapoint import DataPoint
import logging

from utils.constants import DATA_OFFSET, ID_SIZE, MAC_LENGTH, COUNTER_OFFSET, COUNTER_SIZE, MAC_KEY, ID_OFFSET, DATA_LEN_OFFSET, MIN_COMMAND_SIZE

import hashlib


class Command(ABC):

    uplink_args: List[DataPoint] = []
    downlink_telem: List[DataPoint] = []

    def __init__(self) -> None:
        self.uplink_buffer_size = sum(
            [datapoint.num_bytes for datapoint in self.uplink_args])
        self.downlink_buffer_size = sum(
            [datapoint.num_bytes for datapoint in self.downlink_telem])

    @abstractmethod
    def _method(self, **kwargs) -> Dict[str, Union[float, int]]:
        pass

    @staticmethod
    def _unpack(data: bytes, datapoint_list: List[DataPoint]) -> Dict[str, Any]:
        offset = 0
        kwargs = {}
        for point in datapoint_list:
            kwarg = point.unpack(data, offset)
            kwargs.update(kwarg)
            offset += point.num_bytes

        return kwargs

    @staticmethod
    def _pack(kwargs: Dict[str, Any], datapoints: List[DataPoint], buffer_size: int) -> bytes:
        buffer = bytearray(buffer_size)
        offset = 0

        for name, value in kwargs.items():
            datapoint = [p for p in datapoints if p.name == name][0]
            buffer[offset:offset+datapoint.num_bytes] = datapoint.pack(value)
            offset += datapoint.num_bytes

        return bytes(buffer)

    def unpack_args(self, arg_data: Optional[bytes]) -> Dict[str, Any]:
        return self._unpack(arg_data, self.uplink_args)

    def pack_args(self, kwargs: Dict[str, Any]) -> bytes:
        return self._pack(kwargs, self.uplink_args)

    def unpack_telem(self, arg_data: Optional[bytes]) -> Dict[str, Any]:
        return self._unpack(arg_data, self.downlink_telem)

    def pack_telem(self, kwargs: Dict[str, Any]) -> bytes:
        return self._pack(kwargs, self.downlink_telem)

    def run(self, **kwargs):
        downlink = self._method(**kwargs)
        return downlink


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

    command_list: List[Command]
    uplink_counter: int
    downlink_counter: int

    def unpack_command(self, data: bytes) -> Tuple[Command, Dict]:
        if verify_mac(data):
            # TODO deal with counter
            counter = int.from_bytes(
                data[COUNTER_OFFSET:COUNTER_OFFSET + COUNTER_SIZE], 'big')

            command_id = data[ID_OFFSET]
            data_length = data[DATA_LEN_OFFSET]
            command_data = data[DATA_OFFSET:]

            if data_length != len(command_data):
                raise CommandUnpackingException(
                    f"Data Length and command data discrepancy. Data's actual length is {len(command_data)}, but was said to be {data_length}")

            # get command
            command = self.command_list[command_id]
            kwargs = command.unpack_args(command_data)

            return command, kwargs
        else:
            logging.warning(f"Incorrect MAC, ignoring command {data}")
            raise CommandUnpackingException(
                "MAC discrepancy - not running command")

    def pack_command(self, counter: int, command_id: int, **kwargs) -> bytes:
        command = self.command_list[command_id]
        data_buffer = bytearray(
            MIN_COMMAND_SIZE - MAC_LENGTH + command.uplink_buffer_size)
        data_buffer[:COUNTER_SIZE] = counter.to_bytes(COUNTER_SIZE, 'big')
        data_buffer[ID_OFFSET - MAC_LENGTH: ID_OFFSET -
                    MAC_LENGTH + ID_SIZE] = command_id

        arg_data = command.pack_args(**kwargs)
        data_buffer[DATA_OFFSET - MAC_LENGTH:] = arg_data

        mac = compute_mac(bytes(data_buffer))
        return mac + bytes(data_buffer)
