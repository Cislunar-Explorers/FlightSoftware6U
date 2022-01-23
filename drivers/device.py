from abc import ABC, abstractmethod
import logging
from typing import Any


class Device(ABC):
    driver: Any

    def __init__(self, name: str) -> None:
        self.name: str = name
        self.connected: bool = False

    def connect(self):
        try:
            self._connect_to_hardware()
            result = self._self_test()
        except Exception as e:
            logging.error(f"Unable to connect to {self.name}")
            logging.error(e, exc_info=True)
            self.connected = False
        else:
            logging.info(f"{self.name} initialized successfully: {result}")
            self.connected = True

    @abstractmethod
    def _connect_to_hardware(self):
        ...

    @abstractmethod
    def _collect_telem(self) -> Any:
        ...

    def collect_telem(self):
        if self.connected:
            try:
                return self._collect_telem()
            except Exception as e:
                logging.error(e, exc_info=True)
                raise  # TODO, return same type as _collect_telem (but dummy) and don't raise
        else:
            logging.warning(
                f"Not connected to device {self.name}, unable to collect telemetry"
            )

            raise RuntimeError(f"Device {self.name} not connected")

    def _self_test(self):
        return self._collect_telem()
