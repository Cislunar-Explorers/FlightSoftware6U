import os
import shutil
from pathlib import Path

from dotenv import dotenv_values


class __Const(object):
    def __init__(self) -> None:
        """Read from dotenv file and setup constants"""
        self.__config = dotenv_values()
        self.__project_path = (Path(__file__).parent / "../..").resolve()

    @property
    def DATA_PATH(self) -> str:
        """Dotenv DATA_PATH
        :return: sql database parent path
        :rtype: str
        """
        if os.path.isabs(self.__config["DATA_PATH"]):
            return self.__config["DATA_PATH"]
        return os.path.join(self.__project_path, self.__config["DATA_PATH"])

    @property
    def DATA_RESET(self) -> bool:
        """Dotenv DATA_RESET

        :return: whether to delete sql database at start
        :rtype: bool
        """
        return self.__config["DATA_RESET"] == "1"

    @property
    def DB_FILE(self) -> str:
        """Based off of dotenv DATA_PATH

        :return: formated database path for sqlalchemy
        :rtype: str
        """
        return "sqlite:///" + os.path.join(self.DATA_PATH, "db.sqlite")

    @property
    def ATT_ADJUST_RUN_BUFFER(self) -> float:
        """Look ahead time to switch to a different state

        :return: difference in seconds
        :rtype: float
        """
        return 10.0

    @property
    def OP_NAV_RUN_BUFFER(self) -> float:
        """Look ahead time to switch to a different state

        :return: difference in seconds
        :rtype: float
        """
        return 10.0


CONST = __Const()


def __setup_files() -> None:
    """Generate directories and files"""
    if CONST.DATA_RESET and os.path.exists(CONST.DATA_PATH):
        shutil.rmtree(CONST.DATA_PATH)
    os.makedirs(CONST.DATA_PATH, exist_ok=True)


__setup_files()
