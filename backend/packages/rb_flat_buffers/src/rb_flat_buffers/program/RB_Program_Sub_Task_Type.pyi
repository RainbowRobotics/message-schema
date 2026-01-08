from __future__ import annotations

import flatbuffers
import numpy as np

import flatbuffers
import typing

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class RB_Program_Sub_Task_Type(object):
  SUB_TASK_INSERT: int
  SUB_TASK_CHANGE: int

