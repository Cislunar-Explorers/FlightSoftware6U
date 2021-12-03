import logging
import random
import time
from datetime import datetime

import db.models as models
import sqlalchemy as sa

# factory for creating sessions
Session = sa.orm.sessionmaker(models.engine)

# TODO look into Session.future or scoped session?


def add_random_flight_data_rows(n=1):
    with Session.begin() as session:
        logging.info("Add new Flight Data row")
        time.sleep(0.5)
        new_rows = [
            models.FlightData(time=datetime.now(), number=random.randint(1, 101))
            for _ in range(n)
        ]
        session.add_all(new_rows)
    # session.commit() and session.close()


def add_opnav_row(time, number):
    with Session.begin() as session:
        row = models.OpNavData(time=time, number=number)
        session.add(row)


def query_all_flight_data():
    with Session() as session:
        logging.info("Show all Flight Data rows")
        result = session.query(models.FlightData).all()

        for row in result:
            print(f"name: {row.name}, number: {row.number}, complete: {row.complete}")
    # session.close()


def query_all_opnav_data():
    with Session() as session:
        logging.info("Show all OpNav Data rows")
        result = session.query(models.OpNavData).all()

        for row in result:
            print(f"name: {row.name}, number: {row.number}")


def query_latest_opnav_data():
    with Session() as session:
        logging.info("Get latest op-nav row")
        result = session.query(
            models.OpNavData, sa.func.max(models.OpNavData.time)
        ).one()
        if result[1] is not None:
            print(f"min: {result[0].time}, {result[0].number}")
        else:
            print("min: None")
        return result[0] if result[0] is not None else None
    # session.close()


# TODO op nave gets last row of flight data


def query_earliest_flight_data():
    with Session() as session:
        logging.info("Show min Flight Data row")
        time.sleep(0.5)
        # TODO figure out max
        result = (
            session.query(sa.func.min(models.FlightData.number))
            .filter(models.FlightData.complete.is_(False))
            .one()
        )
        return result[0] if result is not None else None
