from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from IPC.FileAck import FileAck
from IPC.FileChunk import FileChunk
from IPC.FileMeta import FileMeta
from IPC.FileRequest import FileRequest
from flatbuffers import table
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class FilePayload(object):
  NONE = cast(int, ...)
  FileMeta = cast(int, ...)
  FileChunk = cast(int, ...)
  FileAck = cast(int, ...)
  FileRequest = cast(int, ...)
def FilePayloadCreator(union_type: typing.Literal[FilePayload.NONE, FilePayload.FileMeta, FilePayload.FileChunk, FilePayload.FileAck, FilePayload.FileRequest], table: table.Table) -> typing.Union[None, FileMeta, FileChunk, FileAck, FileRequest]: ...

