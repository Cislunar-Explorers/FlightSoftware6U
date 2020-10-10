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


class OpNavCoordinatesModel(SQLAlchemyTableBase):
    __tablename__ = "opnav_coordinates"

    id = Column(Integer, primary_key=True)
    time_retrieved = Column(DateTime)
    velocity_x = Column(Float)
    velocity_y = Column(Float)
    velocity_z = Column(Float)
    position_x = Column(Float)
    position_y = Column(Float)
    position_z = Column(Float)
    attitude_x = Column(Float)
    attitude_y = Column(Float)
    attitude_z = Column(Float)

    @staticmethod
    def from_tuples(opnav_tuple, time):
        position, velocity, attitude = opnav_tuple
        velocity_x, velocity_y, velocity_z = velocity
        position_x, position_y, position_z = position
        attitude_x, attitude_y, attitude_z = attitude
        return OpNavCoordinatesModel(
            time_retrieved=time,
            velocity_x=velocity_x,
            velocity_y=velocity_y,
            velocity_z=velocity_z,
            position_x=position_x,
            position_y=position_y,
            position_z=position_z,
            attitude_x=attitude_x,
            attitude_y=attitude_y,
            attitude_z=attitude_z,
        )

    def __repr__(self):
        return (
            f"<OpNavCoorindatesModel(TimeRetrieved=({str(self.time_retrieved)}"
            f", velocity=({self.velocity_x}, {self.velocity_y}, "
            f"{self.velocity_z}) position=({self.position_x}, "
            f"{self.position_y}, {self.position_z}), "
            f"attitude=({self.attitude_x}, {self.attitude_y}, "
            f"{self.attitude_z}))>"
        )


def create_sensor_tables(engine):
    SQLAlchemyTableBase.metadata.create_all(engine)
    create_session.configure(bind=engine)
    return create_session


def create_sensor_tables_from_path(path: str):
    engine = create_engine(path)
    return create_sensor_tables(engine)


class RebootsModel(SQLAlchemyTableBase):
    __tablename__ = "Reboot Directory"

    id = Column(Integer, primary_key=True)
    type_of_reboot = Column(String)
    reboot_at = Column(DateTime)

    def __repr__(self):
        return f"<RebootsModel(type_of_reboot={self.type_of_reboot}, "
        f"reboot_at={str(self.reboot_at)})>"

