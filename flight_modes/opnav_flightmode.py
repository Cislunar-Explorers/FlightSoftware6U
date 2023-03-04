from datetime import datetime
from time import sleep
import sqlite3
from sqlalchemy.exc import SQLAlchemyError
from multiprocessing import Process

from fsw.utils.constants import FMEnum
from fsw.utils.db import OpNavCoordinatesModel
import logging
from fsw.flight_modes.flight_mode import PauseBackgroundMode


class OpNavMode(PauseBackgroundMode):
    # FIXME: There is another class in flight mode with the same name that is in-use. This class is unused.
    flight_mode_id = FMEnum.OpNav.value

    def __init__(self, main):
        super().__init__(main)
        self.dummy_opnav_result = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def set_return_val(self, returnval):
        self.dummy_opnav_result = returnval

    def acquire_images(self):
        return datetime.now()

    def get_position_velocity_attitude(self):
        return self.dummy_opnav_result

    def run_opnav(self):
        sleep(5.0)
        position_acquired_at = self.acquire_images()
        self.most_recent_result = self.get_position_velocity_attitude()
        opnav_model = OpNavCoordinatesModel.from_tuples(
            self.most_recent_result, position_acquired_at
        )
        try:
            session = self._main.create_session()
            session.add(opnav_model)
            session.commit()
        finally:
            session.close()

    def read_most_recent_result(self):
        if self.most_recent_result is not None:
            return self.most_recent_result
        else:
            try:
                session = self._main.create_session()
                last_measurement = (
                    session.query(OpNavCoordinatesModel)
                    .order_by(OpNavCoordinatesModel.measurement_taken)
                    .first()
                )
                if last_measurement is None:
                    raise Exception("No record of previous OpNav runs")
                return self.dummy_opnav_result
            except (sqlite3.Error, SQLAlchemyError):
                session.close()
                raise Exception("Error while reading most recent OpNav result")

    def run_mode(self):
        if not self.opnav_process.is_alive():
            logging.info("[OPNAV]: Able to run next opnav")
            self._main.last_opnav_run = datetime.now()
            logging.info("[OPNAV]: Starting opnav subprocess")
            self.opnav_process = Process(target=self.opnav_subprocess, args=())
            self.opnav_process.start()
        self.run_opnav()
        self.task_completed()

    def update_state(self,sim_input=None) -> int:
        super_fm = super().update_state()

        if super_fm != 0:
            return super_fm

        # return to normal mode if task completed
        if self.task_completed:
            return FMEnum.Normal.value
