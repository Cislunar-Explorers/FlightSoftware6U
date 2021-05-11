from utils.db import TelemetryModel
from utils.db import create_sensor_tables_from_path
from utils.constants import DB_FILE, DB_ENTRY_LIMIT

MEMORY_DB_PATH = "sqlite://"


def display_model(model):
    """
    Displays all entries in desired sqlalchemyModel
    """
    try:
        create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
        session = create_session()

        entries = session.query(model).all()
        for entry in range(len(entries)):
            print(entries[entry])

        session.close()
    finally:
        pass


def display_model_amount(model, amount):
    """
    Displays amount number of most recent entries in desired sqlalchemyModel
    """
    try:
        create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
        session = create_session()

        entries = session.query(model).all()
        length = len(entries)
        for entry in range(length-amount, length):
            print(entries[entry])

        session.close()
    finally:
        pass


def telemetry_query(datatype, amount):
    """
    Datatype specifies the data that wants to be queried inside of TelemetryModel
    Datatype can be: "GOM", "RTC", "RPI", "GYRO", "THERMO", "PRESSURE", or "ALL"
    Prints amount number of entries in the database for desired datatype
    """
    try:
        create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
        session = create_session()

        if datatype.equals("ALL"):
            entries = session.query(TelemetryModel).all()
            length = len(entries)
            for entry in range(length-amount, length):
                print(entries[entry])
        elif datatype.equals("GOM"):
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
                                      )
            length = len(GOM_query)
            for entry in range(length-amount, length):
                print(GOM_query[entry])
        elif datatype.equals("RTC"):
            RTC_query = session.query(TelemetryModel.time_polled,
                                      TelemetryModel.RTC_measurement_taken
                                      )
            length = len(RTC_query)
            for entry in range(length-amount, length):
                print(RTC_query[entry])
        elif datatype.equals("RPI"):
            RPI_query = session.query(TelemetryModel.time_polled,
                                      TelemetryModel.RPI_cpu,
                                      TelemetryModel.RPI_ram,
                                      TelemetryModel.RPI_dsk,
                                      TelemetryModel.RPI_tmp,
                                      TelemetryModel.RPI_boot,
                                      TelemetryModel.RPI_uptime,
                                      )
            length = len(RPI_query)
            for entry in range(length-amount, length):
                print(RPI_query[entry])
        elif datatype.equals("GYRO"):
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
                                       )
            length = len(GYRO_query)
            for entry in range(length-amount, length):
                print(GYRO_query[entry])
        elif datatype.equals("THERMO"):
            THERMO_query = session.query(TelemetryModel.time_polled,
                                         TelemetryModel.THERMOCOUPLE_pressure
                                         )
            length = len(THERMO_query)
            for entry in range(length-amount, length):
                print(THERMO_query[entry])
        elif datatype.equals("PRESSURE"):
            PRESSURE_query = session.query(TelemetryModel.time_polled,
                                           TelemetryModel.PRESSURE_pressure
                                           )
            length = len(PRESSURE_query)
            for entry in range(length-amount, length):
                print(PRESSURE_query[entry])

        session.close()
    finally:
        pass


def clean(model, entry_limit=DB_ENTRY_LIMIT):
    """
    'Cleans' a specific database by deleting the oldest entries
    Keeps entry_limit number of entries still in the database
    Model is a sqlalchemyModel
    """
    try:
        create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
        session = create_session()

        entries = session.query(model).all()
        number_to_delete = len(entries) - entry_limit
        for x in range(number_to_delete):
            session.delete(entries[x])

        session.commit()
    finally:
        pass


def wipe(model):
    """
    Wipes (deletes) all entries from the model that is passed in as an argument
    Model is a sqlalchemyModel
    """
    try:
        create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
        session = create_session()

        entries = session.query(model).all()
        for entry in entries:
            session.delete(entry)

        session.commit()
    finally:
        pass
