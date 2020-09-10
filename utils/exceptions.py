class CislunarException(Exception):
    """Exception running CislunarExplorers FlightSoftware"""


class UnknownFlightModeException(CislunarException):
    """Raise when an unknown flight mode is invoked"""

    def __init__(self, fm_id: int):
        super().__init__(f"Unkown flight mode id: {fm_id}")


class SerializationException(CislunarException):
    """Raise when an exception occurs during serialization"""

class DeserializationException(CislunarException):
    """Raise when an exception occurs during deserialization"""

class CommandPackingException(SerializationException):
    """Raise when an exception occurs while packing a command"""

class CommandUnpackingException(DeserializationException):
    """Raise when an exception occurs while unpacking a command"""

class DownlinkDataPackingException(SerializationException):
    """Raise when an exception occurs while packing downlink data"""

class DownlinkDataUnpackingException(DeserializationException):
    """Raise when an exception occurs while unpacking downlink data"""