import logging
import random
import threading
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
    assert now + timedelta(seconds=15) == res2.time


def main_test():
    main_satellite = MainSatellite()

    now = datetime.now()
    db.add_op_nav_run(now + timedelta(seconds=3))
    db.add_op_nav_run(now + timedelta(seconds=15))
    # db.add_att_adjust_data(now + timedelta(seconds=5))

    threads = [
        threading.Thread(name="main-sat", target=main_satellite.run, daemon=True)
    ]

    for thread in threads:
        thread.start()

    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        logging.info("Keyboard Interrupt received; exiting")


if __name__ == "__main__":
    log.set_up_logging()
    main_test()
    # tests = [
    #     ("General functions", test_db_functions),
    #     ("Att adjust functions", test_db_att_adjust),
    # ]

    # for (test_name, test_func) in tests:
    #     print(f">>>{test_name}")
    #     test_func()
    #     print("---\n")

    # op_nav mode addds new event to queue after completion

    """
    main loops, updating some sensor data every 3 sec
        needs ability to be the only thread running (lock)
    op_nav needs access to data
        needs ability to be the only thread running
        requires knowledge of propulsion events

    each tries to acquire lock A. waits until lock B


    goal for tomorrow: main writend checks for most recent op_nav result
    op_nav read main data and writes
    """
    # EXPECT: 3 added rows, 2 failed status
