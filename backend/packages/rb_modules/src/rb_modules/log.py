from datetime import (
    UTC,
    datetime,
)

from rb_flat_buffers.IPC.State_Log import (
    State_LogT,
)
from rb_zenoh.client import (
    ZenohClient,
)

zenoh_client = ZenohClient()


class RBLog:
    def __init__(self):
        self._log_colors = {
            "INFO": "\033[92m",
            "WARNING": "\033[93m",
            "ERROR": "\033[91m",
            "USER": "\033[95m",
            "DEBUG": "\033[94m",
            "GENERAL": "\033[90m",
            "RESET": "\033[0m",
        }

        self._log_numbers = {
            "INFO": 0,
            "WARNING": 1,
            "ERROR": 2,
            "USER": 3,
            "DEBUG": 4,
            "GENERAL": 5,
        }

    def info(self, message: str, disable_db: bool = False):
        self._print(message, "INFO")
        self._publish_zenoh(message, "INFO", disable_db=disable_db)

    def warning(self, message: str, *, disable_db: bool = False):
        self._print(message, "WARNING")
        self._publish_zenoh(message, "WARNING", disable_db=disable_db)

    def error(self, message: str, *, disable_db: bool = False):
        self._print(message, "ERROR")
        self._publish_zenoh(message, "ERROR", disable_db=disable_db)

    def user(self, message: str, *, disable_db: bool = False):
        self._print(message, "USER")
        self._publish_zenoh(message, "USER", disable_db=disable_db)

    def debug(self, message: str, *, disable_db: bool = True):
        self._print(message, "DEBUG")
        self._publish_zenoh(message, "DEBUG", disable_db=disable_db)

    def general(self, message: str, *, disable_db: bool = False):
        self._print(message, "GENERAL")
        self._publish_zenoh(message, "GENERAL", disable_db=disable_db)

    def _print(self, message: str, level: str):
        print(
            f"{self._log_colors[level]}[{level}] {message}{self._log_colors['RESET']}", flush=True
        )

    def _publish_zenoh(self, message: str, level: str, *, disable_db: bool = False):
        req = State_LogT()
        req.level = self._log_numbers[level]
        req.contents = message
        req.timestamp = datetime.now(UTC).strftime("%H:%M:%S")

        if not disable_db:
            zenoh_client.publish("rrs_log", flatbuffer_req_obj=req, flatbuffer_buf_size=1024)


rb_log = RBLog()
