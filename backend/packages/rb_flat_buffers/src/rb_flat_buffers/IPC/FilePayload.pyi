from __future__ import annotations

import flatbuffers
import numpy as np

import flatbuffers
import typing
from rb_flat_buffers.IPC.FileAck import FileAck
from rb_flat_buffers.IPC.FileChunk import FileChunk
from rb_flat_buffers.IPC.FileMeta import FileMeta
from rb_flat_buffers.IPC.FilePayload import FilePayload
from rb_flat_buffers.IPC.FileRequest import FileRequest
from flatbuffers import table

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class FilePayload(object):
  NONE: int
  FileMeta: int
  FileChunk: int
  FileAck: int
  FileRequest: int
def FilePayloadCreator(union_type: typing.Literal[FilePayload.NONE, FilePayload.FileMeta, FilePayload.FileChunk, FilePayload.FileAck, FilePayload.FileRequest], table: table.Table) -> typing.Union[None, FileMeta, FileChunk, FileAck, FileRequest]: ...

