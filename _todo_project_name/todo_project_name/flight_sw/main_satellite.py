import enum
import logging
import random
import threading
import time
from datetime import datetime

import utils.globals as globals
from db import db, models
from utils.constants import CONST

from todo_project_name.op_nav.op_nav import OpNav

# import sched
# TODO database size
# TODO time comparisons add small epsilon in comparisions


class FlightMode(enum.Enum):
    Normal = enum.auto()
    AttAdjust = enum.auto()
    OpNav = enum.auto()

    def __str__(self) -> str:
        return f"FlightMode.{self.name}({self.value})"


class MainSatellite:
    def __init__(self):
        logging.info("Main satellite initialized")
        self.mode: FlightMode = FlightMode.Normal

    def run(self):
        # init params & sensors
        while True:
            # do not run if global_lock is locked
            with globals.run_camera_cond:
                while globals.want_to_run_camera:
                    logging.info("Can to run camera set to TRUE")
                    globals.can_run_camera = True
                    globals.run_camera_cond.notify_all()
                    globals.run_camera_cond.wait()
                globals.can_run_camera = False
            logging.info(f"Flight mode: {self.mode}")  # Can to run camera set to FALSE.

            if self.mode == FlightMode.Normal:
                db.add_flight_data(number=random.randint(1, 101))
                # db.query_latest_op_nav_data()
                time.sleep(1)
                # if random.randint(0, 2) % 2:
                #     db.add_att_adjust_data(
                #         datetime.now() + timedelta(seconds=random.randint(8, 17))
                #     )
            elif self.mode == FlightMode.AttAdjust:
                att_adjust_run = db.query_earliest_att_adjust_run()
                if att_adjust_run is not None:
                    time_diff = (att_adjust_run.time - datetime.now()).total_seconds()
                    if time_diff > 0:
                        time.sleep(time_diff)
                    with globals.global_lock:
                        logging.info("Begin attitude adjustment")
                        time.sleep(2)
                        db.update_op_nav_run(
                            att_adjust_run.id, models.RunState.SUCCEEDED
                        )
                        logging.info("Complete attitude adjustment")
            elif self.mode == FlightMode.OpNav:
                # db.add_future_op_nav_run()
                # time.sleep(2)
                db.update_failed_op_nav_runs()
                op_nav_run = db.query_earliest_op_nav_run()
                if op_nav_run is not None:
                    op_nav = OpNav()
                    time_diff = (op_nav_run.time - datetime.now()).total_seconds()
                    logging.info(f"Op-nav run scheduled in {time_diff}s")
                    t = threading.Timer(max(time_diff, 0), op_nav.run, (op_nav_run.id,))
                    t.name = "op-nav"
                    t.start()
                    db.update_op_nav_run(op_nav_run.id, models.RunState.RUNNING)
                    # :( scheduler is not threaded
                    # sch = sched.scheduler(time.time, time.sleep)
                    # sch.enterabs(
                    #     op_nav_run.time.timestamp(), 1, op_nav.run, (op_nav_run.id,)
                    # )
                    # sch.run()

            self.transition()

    def transition(self) -> None:
        db.update_failed_att_adjust_runs()
        att_adjust_run = db.query_earliest_att_adjust_run()
        if att_adjust_run is not None:
            time_diff = (att_adjust_run.time - datetime.now()).total_seconds()
            if time_diff >= CONST.ATT_ADJUST_RUN_BUFFER:
                logging.info(f"Next adjustment too far away; time diff: {time_diff}")
            else:
                self.mode = FlightMode.AttAdjust
                return

        db.update_failed_op_nav_runs()
        op_nav_run = db.query_earliest_op_nav_run()
        if op_nav_run is not None:
            time_diff = (op_nav_run.time - datetime.now()).total_seconds()
            if time_diff >= CONST.OP_NAV_RUN_BUFFER:
                logging.info(f"Next adjustment too far away; time diff: {time_diff}")
            else:
                self.mode = FlightMode.OpNav
                return

        self.mode = FlightMode.Normal
