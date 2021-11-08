from __future__ import annotations

from abc import ABC, abstractmethod
from utils.constants import MIN_COMMAND_SIZE
from utils.exceptions import CommandException
from typing import TYPE_CHECKING, Union, Dict, List, Any, Optional
from communications.codec import Codec
import logging

from utils.log import log_error

if TYPE_CHECKING:
    from main import MainSatelliteThread


class Command(ABC):
    uplink_args: List[Codec]
    downlink_telem: List[Codec]
    id: int  # ID must be between 0 and 255 - every command ID must be different. I can't think of a way to autogenerate these, so this will have to be enforced in practice

    def __init__(self) -> None:
        self.uplink_buffer_size = sum(
            [codec.num_bytes for codec in self.uplink_args])
        if self.uplink_buffer_size > 200 - MIN_COMMAND_SIZE:
            logging.error(
                f"Buffer size too big: {self.uplink_buffer_size} > {200 - MIN_COMMAND_SIZE}")
        self.downlink_buffer_size = sum(
            [codec.num_bytes for codec in self.downlink_telem])

    @abstractmethod
    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> Optional[Dict[str, Union[float, int]]]:
        pass

    @staticmethod
    def _unpack(data: bytes, codec_list: List[Codec]) -> Dict[str, Any]:
        offset = 0
        kwargs = {}
        for point in codec_list:
            kwarg = point.unpack(data[offset:offset+point.num_bytes])
            kwargs.update(kwarg)
            offset += point.num_bytes

        return kwargs

    @staticmethod
    def _pack(kwargs: Dict[str, Any], codecs: List[Codec], buffer_size: int) -> bytes:
        buffer = bytearray(buffer_size)
        offset = 0

        for name, value in kwargs.items():
            codec = [p for p in codecs if p.name == name][0]
            buffer[offset:offset+codec.num_bytes] = codec.pack(value)
            offset += codec.num_bytes

        return bytes(buffer)

    def unpack_args(self, arg_data: bytes) -> Dict[str, Any]:
        return self._unpack(arg_data, self.uplink_args)

    def pack_args(self, **kwargs) -> bytes:
        return self._pack(kwargs, self.uplink_args, self.uplink_buffer_size)

    def unpack_telem(self, arg_data: bytes) -> Dict[str, Any]:
        return self._unpack(arg_data, self.downlink_telem)

    def pack_telem(self, **kwargs) -> bytes:
        return self._pack(kwargs, self.downlink_telem, self.downlink_buffer_size)

    def run(self,  parent: Optional[MainSatelliteThread] = None, **kwargs):
        try:
            downlink = self._method(parent=parent, **kwargs)
            return downlink
        except Exception as e:
            logging.error("Unhandled command exception")
            log_error(e, logging.error)
            raise CommandException
