import logging

import sqlalchemy as sa
from sqlalchemy.ext.declarative import declarative_base
from utils.constants import CONST

engine = sa.create_engine(CONST.DB_FILE, echo=False)  # set to True for logging
Base = declarative_base()


class FlightData(Base):
    __tablename__ = "flight_data"
    id = sa.Column(sa.Integer, primary_key=True)

    time = sa.Column(sa.DateTime, nullable=False)
    number = sa.Column(sa.Integer, nullable=False)
    complete = sa.Column(sa.Boolean, nullable=False)

    def __init__(self, *, time, number):
        logging.info(f"Add new flight row: {time}, {number}")
        self.time = time
        self.number = number
        self.complete = False


class OpNavData(Base):
    __tablename__ = "op_nav_data"
    id = sa.Column(sa.Integer, primary_key=True)

    time = sa.Column(sa.DateTime, nullable=False)
    number = sa.Column(sa.Integer, nullable=False)

    def __init__(self, *, time, number):
        logging.info(f"Add new op-nav row: {time}, {number}")
        self.time = time
        self.number = number


# creates all defined table objects into metadata
Base.metadata.create_all(engine)
