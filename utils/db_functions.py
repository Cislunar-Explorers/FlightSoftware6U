from typing import Dict

from utils.db import TelemetryModel, CommandModel, RebootsModel
from utils.db import OpNavTrajectoryStateModel, OpNavAttitudeStateModel, OpNavPropulsionModel
from utils.db import OpNavEphemerisModel, OpNavCameraMeasurementModel, OpNavGyroMeasurementModel
from utils.db import SQLAlchemyTableBase
from utils.db import create_sensor_tables_from_path
from utils.constants import DB_ENTRY_LIMIT
from utils.log import log as logger

# TODO change MEMORY_DB_PATH back to DB_FILE
MEMORY_DB_PATH = "sqlite://"

# each key is the "__tablename__" for each model in db.py
databases_dict: Dict[str, SQLAlchemyTableBase] = {"Commands": CommandModel,
                                                  "opnav_trajectory_state": OpNavTrajectoryStateModel,
                                                  "opnav_attitude_state": OpNavAttitudeStateModel,
                                                  "opnav_ephemeris": OpNavEphemerisModel,
                                                  "opnav_camera_measurement_state": OpNavCameraMeasurementModel,
                                                  "opnav_gyro_measurement_state": OpNavGyroMeasurementModel,
                                                  "opnav_propulsion_state": OpNavPropulsionModel,
                                                  "Reboots": RebootsModel,
                                                  "Telemetry": TelemetryModel
                                                  }


def display_model(model, session):
    """
    Displays all entries in desired sqlalchemyModel
    Model must be a key in the databases_dict corresponding to a model
    A sqlalchemy session (session) must be passed in
    """
    try:
        entries = session.query(databases_dict.get(model)).all()
        for entry in range(len(entries)):
            logger.info(entries[entry])
    except:
        logger.error("error during display_model")


def display_model_amount(model, amount, session):
    """
    Displays amount number of most recent entries in desired sqlalchemyModel
    A sqlalchemy session (session) must be passed in
    """
    try:
        entries = session.query(databases_dict.get(model)).all()
        length = len(entries)
        for entry in range(length - amount, length):
            logger.info(entries[entry])
    except:
        logger.error("error during display_model_amount")


def telemetry_query(datatype, amount, session):
    """
    Datatype specifies the data that wants to be queried inside of TelemetryModel
    Datatype can be: "GOM", "RTC", "RPI", "GYRO", "THERMO", "PRESSURE", or "ALL"
    Prints amount number of most recent entries in the database for desired datatype
    A sqlalchemy session (session) must be passed in
    """
    try:
        if datatype == "ALL":
            entries = session.query(TelemetryModel).all()
            length = len(entries)
            for entry in range(length - amount, length):
                logger.info(entries[entry])
        elif datatype == "GOM":
            GOM_query = session.query(TelemetryModel.time_polled,
                                      TelemetryModel.GOM_vboost1,
                                      TelemetryModel.GOM_vboost2,
                                      TelemetryModel.GOM_vboost3,
                                      TelemetryModel.GOM_vbatt,
                                      TelemetryModel.GOM_curin1,
                                      TelemetryModel.GOM_curin2,
                                      TelemetryModel.GOM_curin3,
                                      TelemetryModel.GOM_cursun,
                                      TelemetryModel.GOM_cursys,
                                      TelemetryModel.GOM_reserved1,
                                      TelemetryModel.GOM_curout1,
                                      TelemetryModel.GOM_curout2,
                                      TelemetryModel.GOM_curout3,
                                      TelemetryModel.GOM_curout4,
                                      TelemetryModel.GOM_curout5,
                                      TelemetryModel.GOM_curout6,
                                      TelemetryModel.GOM_outputs,
                                      TelemetryModel.GOM_latchup1,
                                      TelemetryModel.GOM_latchup2,
                                      TelemetryModel.GOM_latchup3,
                                      TelemetryModel.GOM_latchup4,
                                      TelemetryModel.GOM_latchup5,
                                      TelemetryModel.GOM_latchup6,
                                      TelemetryModel.GOM_wdt_i2c_time_left,
                                      TelemetryModel.GOM_wdt_gnd_time_left,
                                      TelemetryModel.GOM_counter_wdt_i2c,
                                      TelemetryModel.GOM_counter_wdt_gnd,
                                      TelemetryModel.GOM_counter_boot,
                                      TelemetryModel.GOM_bootcause,
                                      TelemetryModel.GOM_battmode,
                                      TelemetryModel.GOM_temp1,
                                      TelemetryModel.GOM_temp2,
                                      TelemetryModel.GOM_temp3,
                                      TelemetryModel.GOM_temp4,
                                      TelemetryModel.GOM_pptmode,
                                      TelemetryModel.GOM_reserved2,
                                      ).all()
            length = len(GOM_query)
            for entry in range(length - amount, length):
                logger.info(GOM_query[entry])
        elif datatype == "RTC":
            RTC_query = session.query(TelemetryModel.time_polled,
                                      TelemetryModel.RTC_measurement_taken
                                      ).all()
            length = len(RTC_query)
            for entry in range(length - amount, length):
                logger.info(RTC_query[entry])
        elif datatype == "RPI":
            RPI_query = session.query(TelemetryModel.time_polled,
                                      TelemetryModel.RPI_cpu,
                                      TelemetryModel.RPI_ram,
                                      TelemetryModel.RPI_dsk,
                                      TelemetryModel.RPI_tmp,
                                      TelemetryModel.RPI_boot,
                                      TelemetryModel.RPI_uptime,
                                      ).all()
            length = len(RPI_query)
            for entry in range(length - amount, length):
                logger.info(RPI_query[entry])
        elif datatype == "GYRO":
            GYRO_query = session.query(TelemetryModel.time_polled,
                                       TelemetryModel.GYRO_gyr_x,
                                       TelemetryModel.GYRO_gyr_y,
                                       TelemetryModel.GYRO_gyr_z,
                                       TelemetryModel.GYRO_acc_x,
                                       TelemetryModel.GYRO_acc_y,
                                       TelemetryModel.GYRO_acc_z,
                                       TelemetryModel.GYRO_mag_x,
                                       TelemetryModel.GYRO_mag_y,
                                       TelemetryModel.GYRO_mag_z,
                                       TelemetryModel.GYRO_temperature
                                       ).all()
            length = len(GYRO_query)
            for entry in range(length - amount, length):
                logger.info(GYRO_query[entry])
        elif datatype == "THERMO":
            THERMO_query = session.query(TelemetryModel.time_polled,
                                         TelemetryModel.THERMOCOUPLE_temperature
                                         ).all()
            length = len(THERMO_query)
            for entry in range(length - amount, length):
                logger.info(THERMO_query[entry])
        elif datatype == "PRESSURE":
            PRESSURE_query = session.query(TelemetryModel.time_polled,
                                           TelemetryModel.PRESSURE_pressure
                                           ).all()
            length = len(PRESSURE_query)
            for entry in range(length - amount, length):
                logger.info(PRESSURE_query[entry])
    except:
        logger.error("error during telemetry_query")


def clean(model, session, entry_limit=DB_ENTRY_LIMIT):
    """
    'Cleans' a specific database by deleting the oldest entries
    Keeps entry_limit number of entries still in the database
    Model is a sqlalchemyModel
    A sqlalchemy session (session) must be passed in
    """
    try:
        entries = session.query(databases_dict.get(model)).all()
        number_to_delete = len(entries) - entry_limit
        for x in range(number_to_delete):
            session.delete(entries[x])
    except:
        logger.error("error during clean")


def wipe(model, session):
    """
    Wipes (deletes) all entries from the model that is passed in as an argument
    Model is a sqlalchemyModel
    A sqlalchemy session (session) must be passed in
    """
    try:
        entries = session.query(databases_dict.get(model)).all()
        for entry in entries:
            session.delete(entry)
    except:
        logger.error("error during wipe")
