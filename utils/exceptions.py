class CislunarException(Exception):
    """Exception running CislunarExplorers FlightSoftware"""


class UnknownFlightModeException(CislunarException):
    """Raise when an unknown flight mode is invoked"""

    def __init__(self, fm_id: int):
        super().__init__(f"Unkown flight mode id: {fm_id}")


class CommandException(CislunarException):
    """Raise when exception occurs during command handling"""


class CommandArgException(CommandException):
    """Raise when an argument to a command is weird"""


class CommandPackingException(CommandException):
    """Raise when an exception occurs while packing a command"""


class CommandUnpackingException(CommandException):
    """Raise when an exception occurs while unpacking a command"""


class SensorError(CislunarException):
    """Raise when exception occurs with a malfunctioning sensor"""
    pass


class GomSensorError(SensorError):
    pass


class GyroError(SensorError):
    pass


class PressureError(SensorError):
    pass


class ThermocoupleError(SensorError):
    pass


class PiSensorError(SensorError):
    pass


class RtcError(SensorError):
    pass


class PowerException(CislunarException):
    pass


class PowerInputError(PowerException):
    pass


class PowerReadError(PowerException):
    pass
