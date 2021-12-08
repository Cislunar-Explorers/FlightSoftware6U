import enum

import sqlalchemy as sa
from sqlalchemy.ext.declarative import declarative_base
from utils.constants import CONST

engine = sa.create_engine(CONST.DB_FILE, echo=False)  # set to True for logging
Base = declarative_base()

# TODO docstrings


class RunState(enum.Enum):
    QUEUED = enum.auto()
    RUNNING = enum.auto()
    FAILED = enum.auto()
    SUCCEEDED = enum.auto()

    def __str__(self) -> str:
        return f"RunState.{self.name}({self.value})"


class FlightData(Base):
    __tablename__ = "flight_data"
    id = sa.Column(sa.Integer, primary_key=True)

    time = sa.Column(sa.DateTime, nullable=False)
    number = sa.Column(sa.Integer, nullable=False)

    def __init__(self, *, time, number) -> None:
        self.time = time
        self.number = number

    def __str__(self) -> str:
        return f"FlightData({self.time}, {self.number})"


class AttAdjustRun(Base):
    __tablename__ = "att_adjust_run"
    id = sa.Column(sa.Integer, primary_key=True)

    time = sa.Column(sa.DateTime, nullable=False)
    state = sa.Column(sa.Enum(RunState), nullable=False)  # persists string

    def __init__(self, *, time) -> None:
        self.time = time
        self.state = RunState.QUEUED

    def __str__(self) -> str:
        return f"AttAdjustRun({self.time}, {self.state})"


class OpNavData(Base):
    __tablename__ = "op_nav_data"
    id = sa.Column(sa.Integer, primary_key=True)

    time = sa.Column(sa.DateTime, nullable=False)
    number = sa.Column(sa.Integer, nullable=False)

    def __init__(self, *, time, number) -> None:
        self.time = time
        self.number = number

    def __str__(self) -> str:
        return f"OpNavData({self.time}, {self.number})"


class OpNavRun(Base):
    __tablename__ = "op_nav_run"
    id = sa.Column(sa.Integer, primary_key=True)

    time = sa.Column(sa.DateTime, nullable=False)
    state = sa.Column(sa.Enum(RunState), nullable=False)

    def __init__(self, *, time) -> None:
        self.time = time
        self.state = RunState.QUEUED

    def __str__(self) -> str:
        return f"OpNavRun({self.time}, {self.state})"


# creates all defined table objects into metadata
Base.metadata.create_all(engine)
