from sqlalchemy import Column, Integer, String, create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.types import Float, DateTime


create_session = sessionmaker()


# declarative_base for WalletModel used by WalletManager
SQLAlchemyTableBase = declarative_base()

# NOTE do not use foreign key in any of these tables
# TODO implement Model classes for all sensor data to be stored


class CommandModel(SQLAlchemyTableBase):
    __tablename__ = "commands"

    id = Column(Integer, primary_key=True)
    command_received = Column(DateTime)
    name = Column(String)
    app_code = Column(Integer)
    opcode = Column(Integer)
    executed_at = Column(DateTime, nullable=True)

    def __repr__(self):
        return f"<CommandModel(Name={self.name}, app_code={self.app_code}, "
        f"opcode={self.opcode} executed_at="
        f"{str(self.executed_at) if self.executed_at is not None else 'NE'})>"


class PressureModel(SQLAlchemyTableBase):
    __tablename__ = "pressure"

    id = Column(Integer, primary_key=True)
    measurement_taken = Column(DateTime)
    pressure = Column(Float)

    def __repr__(self):
        return f"<PressureModel(pressure={self.pressure}, "
        f"taken_at={str(self.measurement_taken)})>"


class RTCModel(SQLAlchemyTableBase):
    __tablename__ = "rtc"

    id = Column(Integer, primary_key=True)
    measurement_taken = Column(DateTime)
    time_retrieved = Column(DateTime)

    def __repr__(self):
        return f"<RTCModel(TimeRetrieved={str(self.time_retrieved)}, "
        f"taken_at={str(self.time_retrieved)})>"


def create_sensor_tables(engine):
    SQLAlchemyTableBase.metadata.create_all(engine)
    create_session.configure(bind=engine)
    return create_session


def create_sensor_tables_from_path(path: str):
    engine = create_engine(path)
    return create_sensor_tables(engine)
