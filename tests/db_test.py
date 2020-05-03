import datetime

from utils.db import create_sensor_tables_from_path, PressureModel

MEMORY_DB_PATH = "sqlite://"


def test_pressure_model():
    create_session = create_sensor_tables_from_path(MEMORY_DB_PATH)
    session = create_session()
    pressure_measurements = session.query(PressureModel).all()
    assert 0 == len(pressure_measurements)

    measurement_taken = datetime.datetime.now()
    pressure = 15.0

    new_measurement = PressureModel(
        measurement_taken=measurement_taken, pressure=pressure
    )
    session.add(new_measurement)
    session.commit()
    pressure_measurements = session.query(PressureModel).all()

    assert 1 == len(pressure_measurements)
    last_measurement = pressure_measurements[0]
    assert last_measurement.measurement_taken == measurement_taken
    assert last_measurement.pressure == pressure

    session.close()
    session = create_session()

    pressure_measurements = session.query(PressureModel).all()

    assert 1 == len(pressure_measurements)
    last_measurement = pressure_measurements[0]
    assert last_measurement.measurement_taken == measurement_taken
    assert last_measurement.pressure == pressure
