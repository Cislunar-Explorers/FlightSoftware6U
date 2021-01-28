from sqlalchemy import Column, Integer, String, create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.types import Float, DateTime, Boolean


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
    attitude_q1 = Column(Float)
    attitude_q2 = Column(Float)
    attitude_q3 = Column(Float)
    attitude_q4 = Column(Float)
    attitude_rod1 = Column(Float)
    attitude_rod2 = Column(Float)
    attitude_rod3 = Column(Float)
    attitude_b1 = Column(Float)
    attitude_b2 = Column(Float)
    attitude_b3 = Column(Float)
    
    @staticmethod
    def from_tuples(opnav_tuple, time):
        position, velocity, attitude_quat, attitude_rod_bias = opnav_tuple
        velocity_x, velocity_y, velocity_z = velocity
        position_x, position_y, position_z = position
        attitude_q1, attitude_q2, attitude_q3, attitude_q4 = attitude_quat
        attitude_rod1, attitude_rod2, attitude_rod3, attitude_b1, attitude_b2, attitude_b3 = attitude_rod_bias
        return OpNavCoordinatesModel(
            time_retrieved=time,
            velocity_x=velocity_x,
            velocity_y=velocity_y,
            velocity_z=velocity_z,
            position_x=position_x,
            position_y=position_y,
            position_z=position_z,
            attitude_q1=attitude_q1,
            attitude_q2=attitude_q2,
            attitude_q3=attitude_q3,
            attitude_q4=attitude_q4,
            attitude_rod1=attitude_rod1,
            attitude_rod2=attitude_rod2,
            attitude_rod3=attitude_rod3,
            attitude_b1=attitude_b1,
            attitude_b2=attitude_b2,
            attitude_b3=attitude_b3
        )

    def __repr__(self):
        return (
            f"<OpNavCoorindatesModel(TimeRetrieved=({str(self.time_retrieved)}"
            f", velocity=({self.velocity_x}, {self.velocity_y}, "
            f"{self.velocity_z}) position=({self.position_x}, "
            f"{self.position_y}, {self.position_z}), "
            f"attitude=({self.attitude_q1}, {self.attitude_q2}, "
            f"attitude=({self.attitude_q3}, {self.attitude_q4}, "
            f"attitude=({self.attitude_rod1}, {self.attitude_rod2}, "
            f"attitude=({self.attitude_rod3}, {self.attitude_b1}, "
            f"attitude=({self.attitude_b2}, {self.attitude_b3}))>"
        )


class RebootsModel(SQLAlchemyTableBase):
    __tablename__ = "Reboots"

    id = Column(Integer, primary_key=True)
    is_bootup = Column(Boolean)
    reboot_at = Column(DateTime)

    def __repr__(self):
        return f"<RebootsModel(is boot up?={self.is_bootup}, "f"reboot_at={str(self.reboot_at)})>"


class GyroModel(SQLAlchemyTableBase):
    __tablename__ = "9DoF"

    id = Column(Integer, primary_key=True)
    time_polled = Column(DateTime)
    gyr_x = Column(Float)
    gyr_y = Column(Float)
    gyr_z = Column(Float)
    acc_x = Column(Float)
    acc_y = Column(Float)
    acc_z = Column(Float)
    mag_x = Column(Float)
    mag_y = Column(Float)
    mag_z = Column(Float)
    temperature = Column(Float)

    @staticmethod
    def from_tuple(gyro_tuple: tuple):
        gyro_data, acc_data, mag_data, temperature, time = gyro_tuple
        gx, gy, gz = gyro_data
        ax, ay, az = acc_data
        bx, by, bz = mag_data
        return GyroModel(
            time_polled=time,
            gyr_x=gx,
            gyr_y=gy,
            gyr_z=gz,
            acc_x=ax,
            acc_y=ay,
            acc_z=az,
            mag_x=bx,
            mag_y=by,
            mag_z=bz,
            temperature=temperature
        )

    def __repr__(self):
        return (
            f"<GyroModel(Name={self.name}"
            f"gyro=({self.gyr_x}, {self.gyr_y}, {self.gyr_z}),"
            f"acc =({self.acc_x}, {self.acc_y}, {self.acc_z}), "
            f"mag =({self.mag_x}, {self.mag_y}, {self.mag_z}),"
            f"temp=({self.temperature})"
            f"time=({self.time_polled}))>"
        )


def create_sensor_tables(engine):
    SQLAlchemyTableBase.metadata.create_all(engine)
    create_session.configure(bind=engine)
    return create_session


def create_sensor_tables_from_path(path: str):
    engine = create_engine(path)
    return create_sensor_tables(engine)
