import logging
import typing
from datetime import datetime

import db.models as models
import sqlalchemy as sa

# factory for creating sessions
Session = sa.orm.sessionmaker(models.engine)

# TODO look into Session.future or scoped session?
# TODO func.min query does it actually return min row??
# TODO use session.begin? expunge row after creation? or session.refresh?


def __add_data(model: models.Base, **kwargs) -> models.Base:
    with Session() as session:
        row = model(**kwargs)
        session.add(row)
        session.commit()
        logging.info(f"Add {model.__tablename__} data: {row}")
    # session.close()
    return row


def add_flight_data(number: int) -> models.FlightData:
    return __add_data(models.FlightData, time=datetime.now(), number=number)


def add_att_adjust_data(time: datetime) -> models.AttAdjustRun:
    return __add_data(models.AttAdjustRun, time=time)


def add_op_nav_data(time: datetime, number: int) -> models.OpNavData:
    return __add_data(models.OpNavData, time=time, number=number)


def add_op_nav_run(time: datetime) -> models.OpNavRun:
    return __add_data(models.OpNavRun, time=time)


def __update_data(model: models.Base, id: int, **kwargs) -> None:
    with Session.begin() as session:
        row = session.query(model).filter(model.id.is_(id)).first()
        if row is not None:
            logging.info(f"Update from {model.__tablename__} data: {row}")
            for (key, val) in kwargs.items():
                setattr(row, key, val)
            logging.info(f"Update to {model.__tablename__} data: {row}")
    # session.commit() and session.close()


def update_flight_data(id: int, number) -> None:
    return __update_data(models.FlightData, id, number=number)


def update_att_adjust_run(id: int, state: models.RunState) -> None:
    return __update_data(models.AttAdjustRun, id, state=state)


def update_op_nav_run(id: int, state: models.RunState) -> None:
    return __update_data(models.OpNavRun, id, state=state)


def __update_failed_runs(model: models.Base) -> int:
    """sets all missed runs to FAILED

    :param model: table to update failed runs
    :type model: models.Base
    :return: the number of missed runs
    :rtype: int
    """
    with Session.begin() as session:
        result = (
            session.query(model)
            .filter(
                model.state.is_(models.RunState.QUEUED), model.time < datetime.now()
            )
            .all()
        )
        for row in result:
            row.state = models.RunState.FAILED
            logging.info(f"Failed {model.__tablename__}: {row}")
        num_failed = len(result)
    # session.commit() and session.close()
    if num_failed > 0:
        logging.info(f"Missed {num_failed} runs")
    return num_failed


def update_failed_att_adjust_runs() -> int:
    return __update_failed_runs(models.AttAdjustRun)


def update_failed_op_nav_runs() -> int:
    return __update_failed_runs(models.OpNavRun)


def __query_all_data(model: models.Base) -> typing.List[models.Base]:
    with Session() as session:
        result = session.query(model).all()
        logging.info(f"Get all {model.__tablename__} data: {len(result)} rows")
        for r in result:
            print("  ", r)
        session.expunge_all()
    # session.close()
    return result


def query_all_flight_data() -> typing.List[models.FlightData]:
    return __query_all_data(models.FlightData)


def query_all_op_nav_data() -> typing.List[models.OpNavData]:
    return __query_all_data(models.OpNavData)


def query_earliest_flight_data() -> typing.Union[models.FlightData, None]:
    with Session() as session:
        result = session.query(
            models.FlightData, sa.func.min(models.FlightData.number)
        ).first()
        session.expunge_all()
    logging.info(f"Get min flight data: {result[0]}")
    return result[0]
    # session.close()


def query_latest_op_nav_data() -> typing.Union[models.OpNavData, None]:
    with Session() as session:
        result = session.query(
            models.OpNavData, sa.func.max(models.OpNavData.time)
        ).first()
        session.expunge_all()
    # session.close()
    logging.info(f"Get latest op-nav data: {result[0]}")
    return result[0]


def __query_earliest(
    model: models.Base,
) -> typing.Union[typing.List[models.Base], None]:
    with Session() as session:
        result = (
            session.query(model, sa.func.min(model.time))
            .filter(model.state.is_(models.RunState.QUEUED))
            .first()
        )
        session.expunge_all()
    # session.close()
    logging.info(f"Get earliest {model.__tablename__}: {result[0]}")
    return result[0]


def query_earliest_att_adjust_run() -> typing.Union[models.AttAdjustRun, None]:
    return __query_earliest(models.AttAdjustRun)


def query_earliest_op_nav_run() -> typing.Union[models.OpNavRun, None]:
    return __query_earliest(models.OpNavRun)
