from __future__ import annotations

from abc import ABC, abstractmethod
from utils.constants import MIN_COMMAND_SIZE, CommandEnum
from utils.exceptions import CommandException
from typing import TYPE_CHECKING, Union, Dict, List, Any, Optional
from communications.codec import Codec
import logging


if TYPE_CHECKING:
    from main import MainSatelliteThread


class Command(ABC):
    """Base class for all commands
    If you want to implement a new command, go to command_definitions.py and add your command there.
    All you need to do if you want to implement a new command is to:
        1. Go to command_definitions.py and make a class inheriting from this class
        2. Override the `self._method` method which can only take in a kwarg dictionary: no args other than `parent`
        3. Add the relevant Codecs - make sure you're unpacking and packing the dict correctly in self._method
        4. Specify the command's `id`. You'll need to add the id to the CommandEnum in utils.constants
        5. Add your new command class to the `COMMAND_LIST` defined at the bottom of command_definitions.py
        6. Make a unit test for your command and add it to tests/commands_test.py
    The rest of this class is used for handling and executing the _method. See also command_handler.py"""

    uplink_codecs: List[Codec]
    downlink_codecs: List[Codec]
    id: CommandEnum  # ID must be between 0 and 255 - every command ID must be different.
    # I can't think of a way to autogenerate these, so this will have to be enforced in practice (and unit test)

    def __init__(self) -> None:
        self.uplink_buffer_size = sum([codec.num_bytes for codec in self.uplink_codecs])
        if self.uplink_buffer_size > 200 - MIN_COMMAND_SIZE:
            logging.error(
                f"Buffer size too big: {self.uplink_buffer_size} > {200 - MIN_COMMAND_SIZE}"
            )
        self.downlink_buffer_size = sum(
            [codec.num_bytes for codec in self.downlink_codecs]
        )

    @abstractmethod
    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Optional[Dict[str, Union[float, int]]]:
        ...

    @staticmethod
    def _unpack(data: bytes, codec_list: List[Codec]) -> Dict[str, Any]:
        offset = 0
        kwargs = {}
        for point in codec_list:
            kwarg = point.unpack(data[offset : offset + point.num_bytes])
            kwargs.update(kwarg)
            offset += point.num_bytes

        return kwargs

    @staticmethod
    def _pack(kwargs: Dict[str, Any], codecs: List[Codec], buffer_size: int) -> bytes:
        buffer = bytearray(buffer_size)
        offset = 0

        for name, value in kwargs.items():
            codec = [p for p in codecs if p.name == name][0]
            buffer[offset : offset + codec.num_bytes] = codec.pack(value)
            offset += codec.num_bytes

        return bytes(buffer)

    def unpack_args(self, arg_data: bytes) -> Dict[str, Any]:
        return self._unpack(arg_data, self.uplink_codecs)

    def pack_args(self, **kwargs) -> bytes:
        return self._pack(kwargs, self.uplink_codecs, self.uplink_buffer_size)

    def unpack_telem(self, arg_data: bytes) -> Dict[str, Any]:
        return self._unpack(arg_data, self.downlink_codecs)

    def pack_telem(self, **kwargs) -> bytes:
        return self._pack(kwargs, self.downlink_codecs, self.downlink_buffer_size)

    def packing_check(self, telem: Optional[Dict], codec_list: List[Codec]):
        error = False
        downlink_name_dict = {codec.name: None for codec in codec_list}
        if telem is None:
            if len(downlink_name_dict) != 0:
                error = True
        elif telem.keys() != downlink_name_dict.keys():
            error = True

        if error:
            logging.error(
                f"The packed data does not have the dictionary keys that are defined in the codec list. \
                    Double-check the implementation of _method and uplink_codecs and downlink_codecs are \
                         consistent in {self.id}"
            )
            raise CommandException

    def run(self, parent: Optional[MainSatelliteThread] = None, **kwargs):
        try:
            downlink = self._method(parent=parent, **kwargs)
            self.packing_check(downlink, self.downlink_codecs)
            return downlink
        except Exception as e:
            logging.error(f"Unhandled exception running command {self.id}")
            logging.error(e, exc_info=True)
            raise CommandException
