from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class RB_Program_Log_Type(object):
  INFO = cast(int, ...)
  WARNING = cast(int, ...)
  ERROR = cast(int, ...)
  USER = cast(int, ...)
  DEBUG = cast(int, ...)
  GENERAL = cast(int, ...)

