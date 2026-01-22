from __future__ import annotations

import flatbuffers
import numpy as np

import flatbuffers
import typing

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class RB_Program_Log_Type(object):
  INFO: int
  WARNING: int
  ERROR: int
  USER: int
  DEBUG: int
  GENERAL: int

