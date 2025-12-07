from __future__ import annotations

import flatbuffers
import numpy as np

import flatbuffers
import typing

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class RB_Flow_Manager_ProgramState(object):
  IDLE: int
  RUNNING: int
  PAUSED: int
  STOPPED: int
  WAITING: int
  ERROR: int
  COMPLETED: int

