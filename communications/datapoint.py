from utils.struct import packer_dict
from typing import Dict, Any

class DataPoint:
    """DataPoint: a class for storing all the data relevant to packing and unpacking one variable."""
    def __init__(self, name, dtype: str) -> None:
        self.name: str = name
        self.packer = packer_dict[dtype][0]
        self.unpacker = packer_dict[dtype][1]
        self.num_bytes: int = packer_dict[dtype][2]

    def pack(self, value: Any) -> bytes:
        """Takes in a value and converts it into bytes to be sent over radio"""
        buf = bytearray(self.num_bytes)
        self.packer(buf, 0, value)
        return buf

    def unpack(self, to_unpack:bytes) -> Dict[str, Any]:
        """Takes bytes data received over radio and converts it into a variable"""
        return {self.name: self.unpacker(to_unpack, 0)[1]}
