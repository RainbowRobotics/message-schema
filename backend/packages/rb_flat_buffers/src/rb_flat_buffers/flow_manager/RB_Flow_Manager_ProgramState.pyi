from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class RB_Flow_Manager_ProgramState(object):
  IDLE = cast(int, ...)
  RUNNING = cast(int, ...)
  PAUSED = cast(int, ...)
  STOPPED = cast(int, ...)
  WAITING = cast(int, ...)
  ERROR = cast(int, ...)
  COMPLETED = cast(int, ...)
  AFTER_COMPLETED = cast(int, ...)

