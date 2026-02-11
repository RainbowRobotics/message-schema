from rb_sdk.base import RBBaseSDK

from rb_flat_buffers.SLAMNAV.RequestGetFileMeta import RequestGetFileMetaT
from rb_flat_buffers.SLAMNAV.ResponseGetFileMeta import ResponseGetFileMetaT
from rb_flat_buffers.SLAMNAV.RequestGetFileChunk import RequestGetFileChunkT
from rb_flat_buffers.SLAMNAV.ResponseGetFileChunk import ResponseGetFileChunkT

class RBAmrFileSDK(RBBaseSDK):
    """Rainbow Robotics AMR File SDK"""

    async def get_file_meta(self, robot_model: str, file_path: str, preferred_chunk_size: int) -> ResponseGetFileMetaT:
        """
        [File Meta 조회]
        - robot_model: 요청을 보낼 로봇 모델
        - file_path: 파일 경로
        - ResponseGetFileMetaT 객체 반환
        """

        # 1) RequestGetFileMetaT 객체 생성
        req = RequestGetFileMetaT()
        req.file_path = file_path
        req.preferredChunkSize = preferred_chunk_size

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/getFileMeta",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetFileMetaT,
            flatbuffer_buf_size=2048,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call File Meta failed: obj_payload is None")

        return result["obj_payload"]

    async def get_file_chunk(self, robot_model: str, file_path: str, idx: int, chunk_size: int) -> ResponseGetFileChunkT:
        """
        [Map File Chunk 조회]
        - robot_model: 요청을 보낼 로봇 모델
        - file_path: 파일 경로
        - idx: 청크 인덱스
        - chunk_size: 청크 크기
        - ResponseGetFileChunkT 객체 반환
        """

        # 1) RequestGetMapFileChunkT 객체 생성
        req = RequestGetFileChunkT()
        req.file_path = file_path
        req.chunkIndex = idx
        req.chunkSize = chunk_size

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/getFileChunk",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetFileChunkT,
            flatbuffer_buf_size=2048,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call File Chunk failed: obj_payload is None")

        return result["obj_payload"]
