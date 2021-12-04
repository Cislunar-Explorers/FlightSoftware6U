import logging
import random
from datetime import datetime, timedelta

from db import db
from utils import log

from todo_project_name.flight_sw.main_satellite import MainSatellite


def test_db_functions():
    n = random.randint(1, 101)

    flight_row = db.add_flight_data(number=n)
    flight_res = db.query_all_flight_data()
    assert len(flight_res) == 1
    assert flight_row.id == flight_res[0].id

    db.update_flight_data(flight_row.id, number=2 * n)
    flight_res2 = db.query_all_flight_data()
    assert 2 * n == flight_res2[0].number


def test_db_att_adjust():
    now = datetime.now()
    db.add_att_adjust_data(now + timedelta(seconds=-10))
    db.add_att_adjust_data(now + timedelta(seconds=-15))
    db.add_att_adjust_data(now + timedelta(seconds=15))
    res = db.update_failed_att_adjust_runs()
    assert res == 2
    res2 = db.query_earliest_att_adjust_run()
    assert res2 is not None and now + timedelta(seconds=15) == res2.time


def main_test():
    now = datetime.now()
    db.add_att_adjust_data(now + timedelta(seconds=4))
    db.add_op_nav_run(now + timedelta(seconds=2))
    db.add_op_nav_run(now + timedelta(seconds=16))

    th = MainSatellite(daemon=True)
    th.start()

    try:
        th.join()
    except KeyboardInterrupt:
        logging.info("Keyboard Interrupt received; exiting")


if __name__ == "__main__":
    log.set_up_logging()
    main_test()
