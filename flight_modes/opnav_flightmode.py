import logging

# import sqlite3
from datetime import datetime
from multiprocessing import Process

import utils.constants as consts

from .flight_mode import PauseBackgroundMode

# from sqlalchemy.exc import SQLAlchemyError


# from time import sleep


# from utils.db import OpNavCoordinatesModel


class OpNavMode(PauseBackgroundMode):
    """FMID 5: Optical Navigation Flight Mode
    This flight mode is dedicated to starting the Opnav process"""

    flight_mode_id = consts.FMEnum.OpNav.value

    def __init__(self, parent):
        super().__init__(parent)

    def read_most_recent_result(self):
        if self.most_recent_result is not None:
            return self.most_recent_result
        else:
            pass
            # try:
            #     session = self._parent.create_session()
            #     last_measurement = (
            #         session.query(OpNavCoordinatesModel)
            #         .order_by(OpNavCoordinatesModel.measurement_taken)
            #         .first()
            #     )
            #     if last_measurement is None:
            #         raise Exception("No record of previous OpNav runs")
            #     return self.dummy_opnav_result
            # except (sqlite3.Error, SQLAlchemyError):
            #     session.close()
            #     raise Exception("Error while reading most recent OpNav result")

    def run_mode(self):
        # TODO: overhaul so opnav calculation only occur while in opnav mode
        if not self.opnav_process.is_alive():
            logging.info("[OPNAV]: Able to run next opnav")
            self._parent.last_opnav_run = datetime.now()
            logging.info("[OPNAV]: Starting opnav subprocess")
            self.opnav_process = Process(
                target=self.opnav_subprocess, args=(self._parent.opnav_proc_queue,)
            )
            self.opnav_process.start()
        self.run_opnav()
        self.task_completed()

    def update_state(self) -> int:
        super_fm = super().update_state()

        if super_fm != consts.NO_FM_CHANGE:
            return super_fm
        # check if opnav db has been updated, then set self.task_completed true

        # return to normal mode if task completed
        if self.task_completed:
            return consts.NO_FM_CHANGE

    def opnav_subprocess(self, q):
        # TODO put in try...except
        # TODO change from pytest to actual opnav
        # os.system("pytest OpticalNavigation/tests/test_pipeline.py::test_start")
        # subprocess.run('pytest OpticalNavigation/tests/test_pipeline.py::test_start', shell=True)
        pass
        # subprocess.run(
        #     "echo [OPNAV]: Subprocess Start; sleep 1m; echo [OPNAV]: Subprocess end",
        #     shell=True,
        # )
        # q.put("Opnav Finished")
