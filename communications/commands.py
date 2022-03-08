from __future__ import annotations

from abc import ABC, abstractmethod
from utils.constants import MAX_COMMAND_SIZE, MIN_COMMAND_SIZE, CommandEnum
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
        3. Add the relevant codec for every variable that you want to be able to uplink and downlink in the class's
            `uplink_codecs` and `downlink_codecs` lists. Each codec is defined of the name of the variable you want to
            send, and the data type of that variable. (If you don't have any data to up or downlink, don't worry about
            this step.)
        3a. Make sure that every value you define in the uplink_codecs list get unpacked from the `kwargs` dict in the
            `_method`
        3b. Make sure that every value you define in the downlink_codecs list get returned in a dict by the `_method`.
        4. Specify the command's `id`. You'll need to add the id to the `CommandEnum` in utils.constants
        5. Add your new command class to the `COMMAND_LIST` defined at the bottom of command_definitions.py
        6. Make a unit test for your command and add it to tests/commands_test.py
    The rest of this class is used for handling and executing the _method. See also command_handler.py"""

    # This list defines the name and datatype of each command's arguments that gets uplinked and processed by the
    # command_handler. In other words, this defines which variables you want to uplink to the spacecraft and how
    # those variables get packaged from a value (such as 823.153) to bytes (0x444dc9cb,
    # or 0b01000100010011011100100111001011) which get sent over radio.
    uplink_codecs: List[Codec]

    # This list defines the name and datatype of each of the command's return types that get downlinked over radio
    downlink_codecs: List[Codec]

    id: CommandEnum  # ID must be between 0 and 255 - every command ID must be different.
    # I can't think of a way to autogenerate these, so this will have to be enforced in practice (and unit test)

    def __init__(self) -> None:
        self.uplink_buffer_size = sum([codec.num_bytes for codec in self.uplink_codecs])
        if self.uplink_buffer_size > MAX_COMMAND_SIZE - MIN_COMMAND_SIZE:
            logging.error(
                f"Buffer size too big: {self.uplink_buffer_size} > {MAX_COMMAND_SIZE - MIN_COMMAND_SIZE}"
            )
        self.downlink_buffer_size = sum(
            [codec.num_bytes for codec in self.downlink_codecs]
        )
        self.uplink_codecs_dict = {codec.name: codec for codec in self.uplink_codecs}
        self.downlink_codecs_dict = {
            codec.name: codec for codec in self.downlink_codecs
        }

    @abstractmethod
    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Optional[Dict[str, Union[float, int]]]:
        ...

    @staticmethod
    def _unpack(data: bytes, codec_dict: Dict[str, Codec]) -> Dict[str, Any]:
        offset = 0
        kwargs = {}
        for codec in codec_dict.values():
            kwarg = codec.unpack(data[offset : offset + codec.num_bytes])
            kwargs.update(kwarg)
            offset += codec.num_bytes

        return kwargs

    @staticmethod
    def _pack(
        kwargs: Dict[str, Any], codecs_dict: Dict[str, Codec], buffer_size: int
    ) -> bytes:
        buffer = bytearray(buffer_size)
        offset = 0

        for name, value in kwargs.items():
            codec = codecs_dict[name]
            buffer[offset : offset + codec.num_bytes] = codec.pack(value)
            offset += codec.num_bytes

        return bytes(buffer)

    def unpack_args(self, arg_data: bytes) -> Dict[str, Any]:
        return self._unpack(arg_data, self.uplink_codecs_dict)

    def pack_args(self, arg_dict) -> bytes:
        return self._pack(arg_dict, self.uplink_codecs_dict, self.uplink_buffer_size)

    def unpack_telem(self, arg_data: bytes) -> Dict[str, Any]:
        return self._unpack(arg_data, self.downlink_codecs_dict)

    def pack_telem(self, telem_dict) -> bytes:
        return self._pack(
            telem_dict, self.downlink_codecs_dict, self.downlink_buffer_size
        )

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
                "The packed data does not have the dictionary keys that are defined in the codec list.                "
                "     Double-check the implementation of _method and uplink_codecs and downlink_codecs are            "
                f"              consistent in {self.id}"
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
