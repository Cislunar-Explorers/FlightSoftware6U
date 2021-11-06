from abc import ABC, abstractmethod
from utils.struct import packer_dict
from typing import Union, Dict, List, Any, Optional





if __name__ == '__main__':
    test_point = DataPoint("VBATT", 'short')
    data = test_point.pack(7523)
    print(data)
    result = test_point.unpack(data)
    print(result)



class Command(ABC):

    uplink_args: List[DataPoint] = []
    downlink_telem: List[DataPoint] = []

    def __init__(self) -> None:
        self.uplink_buffer_size = sum([datapoint.num_bytes for datapoint in self.uplink_args])
        self.downlink_buffer_size = sum([datapoint.num_bytes for datapoint in self.downlink_telem])

    @abstractmethod
    def _method(self, **kwargs) -> Dict[str, Union[float, int]]:
        pass

    @staticmethod
    def _unpack(data, datapoint_list: List[DataPoint]) -> Dict[str, Any]:
        offset = 0
        kwargs = {}
        for point in datapoint_list:
            kwarg = point.unpack(data, offset)



        return kwargs

    @staticmethod
    def _pack(kwargs: Dict[str, Any], datapoints: List[DataPoint], buffer_size: int) -> bytes:
        buffer = bytearray(buffer_size)
        for name, value in kwargs.items():
            datapoint = datapoints[name]
            off = datapoint.pack()


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
