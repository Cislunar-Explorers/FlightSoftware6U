from time import time


from utils.db import create_sensor_tables_from_path, TelemetryModel

MEMORY_DB_PATH = "sqlite://"


def test_telemetry_model():
    create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
    session = create_session()
    telemetry_measurements = session.query(TelemetryModel).all()
    assert 0 == len(telemetry_measurements)

    measurement_taken = time()

    new_measurement = TelemetryModel(
        time_polled=measurement_taken,

        # GOM DATA
        GOM_vboost1=1,
        GOM_vboost2=2,
        GOM_vboost3=3,
        GOM_vbatt=4,
        GOM_curin1=5,
        GOM_curin2=6,
        GOM_curin3=7,
        GOM_cursun=8,
        GOM_cursys=9,
        GOM_reserved1=10,
        GOM_curout1=11,
        GOM_curout2=12,
        GOM_curout3=13,
        GOM_curout4=14,
        GOM_curout5=15,
        GOM_curout6=16,
        GOM_outputs=17,
        GOM_latchup1=18,
        GOM_latchup2=19,
        GOM_latchup3=20,
        GOM_latchup4=21,
        GOM_latchup5=22,
        GOM_latchup6=23,
        GOM_wdt_i2c_time_left=24,
        GOM_wdt_gnd_time_left=25,
        GOM_counter_wdt_i2c=26,
        GOM_counter_wdt_gnd=27,
        GOM_counter_boot=28,
        GOM_bootcause=29,
        GOM_battmode=30,
        GOM_temp1=32,
        GOM_temp2=33,
        GOM_temp3=34,
        GOM_temp4=35,
        GOM_pptmode=36,
        GOM_reserved2=37,

        # RTC DATA
        RTC_measurement_taken=38.0,

        # RPi DATA
        RPI_cpu=39,
        RPI_ram=40,
        RPI_dsk=41,
        RPI_tmp=42,
        RPI_boot=43.0,
        RPI_uptime=44.0,

        # GYRO DATA
        GYRO_gyr_x=45.0,
        GYRO_gyr_y=46.0,
        GYRO_gyr_z=47.0,
        GYRO_acc_x=48.0,
        GYRO_acc_y=49.0,
        GYRO_acc_z=50.0,
        GYRO_mag_x=51.0,
        GYRO_mag_y=52.0,
        GYRO_mag_z=53.0,
        GYRO_temperature=54.0,

        # THERMOCOUPLE DATA
        THERMOCOUPLE_pressure=55.0,

        # PRESSURE DATA
        PRESSURE_pressure=56.0
    )

    session.add(new_measurement)
    session.commit()

    # all telemetry measurements
    telemetry_measurements = session.query(TelemetryModel).all()
    assert 1 == len(telemetry_measurements)
    for entry in telemetry_measurements:
        print(type(telemetry_measurements))
        print(telemetry_measurements[0])
        print(entry)

    # just gom query
    GOM_query = session.query(TelemetryModel.time_polled,
                              TelemetryModel.GOM_vboost1,
                              TelemetryModel.GOM_vboost2,
                              TelemetryModel.GOM_vboost3,
                              # more obviously
                              )
    for entry in GOM_query:
        print(entry)


if __name__ == "__main__":
    test_telemetry_model()
