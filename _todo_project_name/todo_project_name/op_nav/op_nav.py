import enum
import logging
import time
from datetime import datetime

import utils.globals as globals
from db import db, models


class OpNavStatus(enum.Enum):
    SUCCESS = enum.auto()
    FAILURE = enum.auto()

    def __str__(self) -> str:
        return f"OpNavStatus.{self.name}({self.value})"


delay = 2


class OpNav:
    def __init__(self):
        logging.info("Op-nav initialized")

    def __observe(self) -> models.FlightData:
        # photo timing should not be stopped
        # with globals.global_lock:
        with globals.run_camera_cond:
            logging.info("Want to run camera set to TRUE")
            globals.want_to_run_camera = True
            globals.run_camera_cond.notify_all()
            while not globals.can_run_camera:
                globals.run_camera_cond.wait()
            logging.info("Camera recording started")
            time.sleep(5)
            res = db.query_earliest_flight_data()
            logging.info("Camera recording completed")

            logging.info("Want to run camera set to FALSE")
            globals.want_to_run_camera = False
            globals.run_camera_cond.notify_all()
        return res

    def __process_data(self, flight_data: models.FlightData):
        time.sleep(2)
        db.add_op_nav_data(datetime.now(), 2 * flight_data.number)

    def run(self, run_id):
        # while query attitude for time < now: sleep()
        logging.info(f"ID: {run_id}")
        logging.info(f"Op-nav run beginning at {datetime.now()}")
        res = self.__observe()
        self.__process_data(res)
        logging.info(f"Op-nav run ending at {datetime.now()}")
        db.update_op_nav_run(run_id, models.RunState.SUCCEEDED)

        # while True:  # op_nav interval
        #     time.sleep(delay)
        #     # TODO check for future attitude adjustment
        #     while True:
        #         op_nav_run = db.query_earliest_op_nav_run()
        #         if op_nav_run is None:
        #             break
        #         time_diff = (op_nav_run.time - datetime.now()).total_seconds()
        #         if time_diff > 2 * delay:
        #             break
        #         if time_diff > 0:
        #             logging.info(f"Upcoming op-nav run. Waiting {time_diff}")
        #             time.sleep(time_diff)
        #         else:
        #             logging.info("Op-nav run time passed")
        #             db.update_op_nav_run(op_nav_run.id, True)

        #     res = self.__observe()
        #     time.sleep(delay)
        #     logging.info(f"Add new op-nav row")
        #     if res is not None:
        #         db.add_op_nav_data(datetime.now(), res)

        return OpNavStatus.SUCCESS
