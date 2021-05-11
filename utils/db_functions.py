from utils.db import TelemetryModel
from utils.db import create_sensor_tables_from_path
from utils.constants import DB_FILE
# import the rest of the models


def telemetry_query(datatype, limit):
    """
    Datatype specifies the data that wants to be queried inside of TelemetryModel
    Datatype can be: "GOM", "RTC", "RPI", "GYRO", "THERMO", "PRESSURE", or "ALL"
    Prints limit number of entries in the database for desired datatype
    """
    try:
        create_session = create_sensor_tables_from_path(DB_FILE)
        session = create_session()
        if datatype.equals("ALL"):
            telemetry_measurements = session.query(TelemetryModel).all()
            for entry in range(limit):
                print(telemetry_measurements[entry])
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
            for entry in range(limit):
                print(GOM_query[entry])
        elif datatype.equals("RTC"):
            RTC_query = session.query(TelemetryModel.time_polled,
                                      TelemetryModel.RTC_measurement_taken
                                      )
            for entry in range(limit):
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
            for entry in range(limit):
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
            for entry in range(limit):
                print(GYRO_query[entry])
        elif datatype.equals("THERMO"):
            THERMO_query = session.query(TelemetryModel.time_polled,
                                         TelemetryModel.THERMOCOUPLE_pressure
                                         )
            for entry in range(limit):
                print(THERMO_query[entry])
        elif datatype.equals("PRESSURE"):
            PRESSURE_query = session.query(TelemetryModel.time_polled,
                                           TelemetryModel.PRESSURE_pressure
                                           )
            for entry in range(limit):
                print(PRESSURE_query[entry])
        session.close()
    finally:
        pass
