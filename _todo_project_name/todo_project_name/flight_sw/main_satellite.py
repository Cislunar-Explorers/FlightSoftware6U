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
# TODO does camera need to block main thread? makes att-adjust lock really weird


class FlightMode(enum.Enum):
    Normal = enum.auto()
    AttAdjust = enum.auto()
    OpNav = enum.auto()

    def __str__(self) -> str:
        return f"FlightMode.{self.name}({self.value})"


class MainSatellite(threading.Thread):
    def __init__(self, **kwargs) -> None:
        threading.Thread.__init__(self, **kwargs)
        self.name = "main-sat"

        logging.info("Main satellite initialized")
        self.mode: FlightMode = FlightMode.Normal
        self.continue_run: bool = True
        self.run_count: int = 0

    def run(self) -> None:
        # init params & sensors
        while self.continue_run:
            self.name = f"main-sat-{self.run_count % 10}"
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
            elif self.mode == FlightMode.AttAdjust:
                db.update_failed_att_adjust_runs()
                att_adjust_run = db.query_earliest_att_adjust_run()
                if att_adjust_run is not None:
                    time_diff = (att_adjust_run.time - datetime.now()).total_seconds()
                    logging.info(f"Att-adjust run scheduled in {time_diff}s")
                    time.sleep(max(time_diff, 0))  # TODO replace with separate thread?
                    logging.info("Begin attitude adjustment")
                    time.sleep(3)
                    db.update_att_adjust_run(
                        att_adjust_run.id, models.RunState.SUCCEEDED
                    )
                    logging.info("Complete attitude adjustment")
            elif self.mode == FlightMode.OpNav:
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

            self.transition()
            self.run_count += 1
            if self.run_count > 15:  # For debugging purposes
                self.continue_run = False

    def transition(self) -> None:
        a_time_diff = -1
        o_time_diff = -1

        db.update_failed_att_adjust_runs()
        att_adjust_run = db.query_earliest_att_adjust_run()
        if att_adjust_run is not None:
            a_time_diff = (att_adjust_run.time - datetime.now()).total_seconds()
            if a_time_diff >= CONST.ATT_ADJUST_RUN_BUFFER:
                logging.info(
                    f"Next att-adjust run too far away; time diff: {a_time_diff}"
                )
            else:
                self.mode = FlightMode.AttAdjust
                return

        db.update_failed_op_nav_runs()
        op_nav_run = db.query_earliest_op_nav_run()
        if op_nav_run is not None:
            o_time_diff = (op_nav_run.time - datetime.now()).total_seconds()
            # while query attitude for time < now: sleep()
            if (
                (att_adjust_run is not None)
                and (
                    a_time_diff >= o_time_diff
                    and a_time_diff <= o_time_diff + CONST.MAX_OP_NAV_CAMERA_TIME
                )
                or (
                    a_time_diff + CONST.MAX_ATT_ADJUST_TIME >= o_time_diff
                    and a_time_diff + CONST.MAX_ATT_ADJUST_TIME
                    <= o_time_diff + CONST.MAX_OP_NAV_CAMERA_TIME
                )
            ):
                # Att-adjust will happen during op-nav camera
                logging.info(
                    "Next op-nav run interferes with att-adjust. Op-nav run canceld."
                )
                db.update_op_nav_run(op_nav_run.id, models.RunState.FAILED)

            # o_time_diff + CONST.MAX_OP_NAV_CAMERA_TIME <= a_time_diff
            if o_time_diff >= CONST.OP_NAV_RUN_BUFFER:
                logging.info(f"Next op-nav run too far away; time diff: {o_time_diff}")
            else:
                self.mode = FlightMode.OpNav
                return

        self.mode = FlightMode.Normal
