from sqlalchemy import Column, Integer, String, create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.types import Float, DateTime, Boolean
from drivers.power.power_structs import eps_hk_t

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
        return f"<CommandModel(Name={self.name}, app_code={self.app_code}, " \
               f"opcode={self.opcode} executed_at=" \
               f"{str(self.executed_at) if self.executed_at is not None else 'NE'})>"


class PressureModel(SQLAlchemyTableBase):
    __tablename__ = "pressure"

    id = Column(Integer, primary_key=True)
    measurement_taken = Column(DateTime)
    pressure = Column(Float)

    def __repr__(self):
        return (f"<PressureModel(pressure={self.pressure}, "
                f"taken_at={str(self.measurement_taken)})>")


class ThermocoupleModel(SQLAlchemyTableBase):
    __tablename__ = "thermocouple"

    id = Column(Integer, primary_key=True)
    measurement_taken = Column(DateTime)
    pressure = Column(Float)


class RTCModel(SQLAlchemyTableBase):
    __tablename__ = "rtc"

    id = Column(Integer, primary_key=True)
    measurement_taken = Column(DateTime)
    time_retrieved = Column(DateTime)

    def __repr__(self):
        return (f"<RTCModel(TimeRetrieved={str(self.time_retrieved)}, "
                f"taken_at={str(self.time_retrieved)})>")


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
        return f"<RebootsModel(is boot up?={self.is_bootup}, reboot_at={str(self.reboot_at)})>"


class GyroModel(SQLAlchemyTableBase):
    __tablename__ = "9DoF"

    id = Column(Integer, primary_key=True)
    time_polled = Column(Float)
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
            f"<GyroModel("
            f"gyr=({self.gyr_x}, {self.gyr_y}, {self.gyr_z}),"
            f"acc=({self.acc_x}, {self.acc_y}, {self.acc_z}), "
            f"mag=({self.mag_x}, {self.mag_y}, {self.mag_z}),"
            f"temp={self.temperature}"
            f"time={self.time_polled})>"
        )


class RPiModel(SQLAlchemyTableBase):
    __tablename__ = "RPi"

    id = Column(Integer, primary_key=True)
    time_polled = Column(DateTime)
    cpu = Column(Integer)
    ram = Column(Integer)
    dsk = Column(Integer)
    tmp = Column(Integer)
    boot = Column(Float)
    uptime = Column(Float)

    @staticmethod
    def from_tuple(rpi_tuple: tuple):
        cpu, ram, dsk, boot_time, uptime, temp, poll_time = rpi_tuple
        temp = int(temp * 10)
        # we save 2 bytes of downlink by converting the rpi temperature to an int (which is then packed as a short
        # during transmission), but we don't lose any accuracy. On the GS we will need to divide the temperature by 10

        return RPiModel(
            time_polled=poll_time,
            cpu=cpu,
            ram=ram,
            dsk=dsk,
            boot=boot_time,
            uptime=uptime,
            tmp=temp
        )

    def __repr__(self):
        return (
            f"<RPiModel("
            f"cpu={self.cpu}, "
            f"ram={self.ram}, "
            f"dsk={self.dsk}, "
            f"temp={self.tmp}, "
            f"boot time={self.boot_time}, "
            f"uptime={self.uptime}, "
            f"poll time={self.time_polled})>"
        )


class GomModel(SQLAlchemyTableBase):
    __tablename__ = "Gom"
    # See drivers/power/power_structs.py line #115 for reference
    id = Column(Integer, primary_key=True)
    time_polled = Column(DateTime)
    vboost1 = Column(Integer)
    vboost2 = Column(Integer)
    vboost3 = Column(Integer)
    vbatt = Column(Integer)
    curin1 = Column(Integer)
    curin2 = Column(Integer)
    curin3 = Column(Integer)
    cursun = Column(Integer)
    cursys = Column(Integer)
    reserved1 = Column(Integer)
    curout1 = Column(Integer)
    curout2 = Column(Integer)
    curout3 = Column(Integer)
    curout4 = Column(Integer)
    curout5 = Column(Integer)
    curout6 = Column(Integer)
    outputs = Column(Integer)
    latchup1 = Column(Integer)
    latchup2 = Column(Integer)
    latchup3 = Column(Integer)
    latchup4 = Column(Integer)
    latchup5 = Column(Integer)
    latchup6 = Column(Integer)
    wdt_i2c_time_left = Column(Integer)
    wdt_gnd_time_left = Column(Integer)
    counter_wdt_i2c = Column(Integer)
    counter_wdt_gnd = Column(Integer)
    counter_boot = Column(Integer)
    bootcause = Column(Integer)
    battmode = Column(Integer)
    temp1 = Column(Integer)
    temp2 = Column(Integer)
    temp3 = Column(Integer)
    temp4 = Column(Integer)
    pptmode = Column(Integer)
    reserved2 = Column(Integer)

    @staticmethod
    def from_struct(eps_hk: eps_hk_t, poll_time):
        return GomModel(
            time_polled=poll_time,
            vboost1=eps_hk.vboost[0],
            vboost2=eps_hk.vboost[1],
            vboost3=eps_hk.vboost[2],
            vbatt=eps_hk.vbatt,
            curin1=eps_hk.curin[0],
            curin2=eps_hk.curin[1],
            curin3=eps_hk.curin[2],
            cursun=eps_hk.cursun,
            cursys=eps_hk.cursys,
            reserved1=eps_hk.reserved1,
            curout1=eps_hk.curout[0],
            curout2=eps_hk.curout[1],
            curout3=eps_hk.curout[2],
            curout4=eps_hk.curout[3],
            curout5=eps_hk.curout[4],
            curout6=eps_hk.curout[5],
            outputs=int(str(eps_hk.output).replace(',', '').replace(' ', '')[1:-1], 2),
            latchup1=eps_hk.latchup[0],
            latchup2=eps_hk.latchup[1],
            latchup3=eps_hk.latchup[2],
            latchup4=eps_hk.latchup[3],
            latchup5=eps_hk.latchup[4],
            latchup6=eps_hk.latchup[5],
            wdt_i2c_time_left=eps_hk.wdt_i2c_time_left,
            wdt_gnd_time_left=eps_hk.wdt_gnd_time_left,
            counter_wdt_i2c=eps_hk.counter_wdt_i2c,
            counter_wdt_gnd=eps_hk.counter_wdt_gnd,
            counter_boot=eps_hk.counter_boot,
            temp1=eps_hk.temp[0],
            temp2=eps_hk.temp[1],
            temp3=eps_hk.temp[2],
            temp4=eps_hk.temp[3],
            bootcause=eps_hk.bootcause,
            battmode=eps_hk.battmode,
            pptmode=eps_hk.pptmode,
            reserved2=eps_hk.reserved2
        )

    def __repr__(self):
        return (f""
                f""
                f""
                f"")
        # TODO


def create_sensor_tables(engine):
    SQLAlchemyTableBase.metadata.create_all(engine)
    create_session.configure(bind=engine)
    return create_session


def create_sensor_tables_from_path(path: str):
    engine = create_engine(path)
    return create_sensor_tables(engine)
