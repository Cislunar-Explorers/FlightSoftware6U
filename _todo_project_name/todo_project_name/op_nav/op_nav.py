import logging
import time
from datetime import datetime

import utils.globals as globals
from db import db, models


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

    def __process_data(self, flight_data: models.FlightData) -> None:
        time.sleep(2)
        db.add_op_nav_data(datetime.now(), 2 * flight_data.number)

    def run(self, run_id) -> None:
        logging.info(f"ID: {run_id}")
        logging.info(f"Op-nav run beginning at {datetime.now()}")
        res = self.__observe()
        self.__process_data(res)
        logging.info(f"Op-nav run ending at {datetime.now()}")
        db.update_op_nav_run(run_id, models.RunState.SUCCEEDED)
