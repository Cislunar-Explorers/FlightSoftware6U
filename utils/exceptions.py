class CislunarException(Exception):
    """Exception running CislunarExplorers FlightSoftware"""


class UnknownFlightModeException(CislunarException):
    """Raise when an unknown flight mode is invoked"""

    def __init__(self, fm_id: int):
        super().__init__(f"Unkown flight mode id: {fm_id}")


class CommandException(CislunarException):
    """Raise when exception occurs during command handling"""


class CommandPackingException(CommandException):
    """Raise when an exception occurs while packing a command"""


class CommandUnpackingException(CommandException):
    """Raise when an exception occurs while unpacking a command"""
