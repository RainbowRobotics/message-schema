from rb_flat_buffers.SLAMNAV.RequestCompressFiles import RequestCompressFilesT
from rb_flat_buffers.SLAMNAV.RequestDecompressFile import RequestDecompressFileT
from rb_flat_buffers.SLAMNAV.RequestDeleteFile import RequestDeleteFileT
from rb_flat_buffers.SLAMNAV.RequestGetFileChunk import RequestGetFileChunkT
from rb_flat_buffers.SLAMNAV.RequestGetFileMeta import RequestGetFileMetaT
from rb_flat_buffers.SLAMNAV.ResponseCompressFiles import ResponseCompressFilesT
from rb_flat_buffers.SLAMNAV.ResponseDecompressFile import ResponseDecompressFileT
from rb_flat_buffers.SLAMNAV.ResponseDeleteFile import ResponseDeleteFileT
from rb_flat_buffers.SLAMNAV.ResponseGetFileChunk import ResponseGetFileChunkT
from rb_flat_buffers.SLAMNAV.ResponseGetFileMeta import ResponseGetFileMetaT

from rb_sdk.base import RBBaseSDK


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
        req.filePath = file_path
        req.preferredChunkSize = preferred_chunk_size

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/getFileMeta",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetFileMetaT,
            flatbuffer_buf_size=2048,
            timeout=100,
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
        req.filePath = file_path
        req.chunkIndex = idx
        req.chunkSize = chunk_size

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/getFileChunk",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetFileChunkT,
            flatbuffer_buf_size=2048,
            timeout=100,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call File Chunk failed: obj_payload is None")

        return result["obj_payload"]

    async def compress_files(self, robot_model: str, base_dir: str, source_names: list[str], output_name: str) -> ResponseCompressFilesT:
        """
        [Files Compress 압축]
        - robot_model: 요청을 보낼 로봇 모델
        - base_dir: 압축할 디렉토리 경로
        - source_names: 압축할 파일 이름 목록
        - output_name: 압축 파일 이름
        - ResponseCompressFilesT 객체 반환
        """

        # 1) RequestCompressFilesT 객체 생성
        req = RequestCompressFilesT()
        req.baseDir = base_dir
        req.sourceNames = source_names
        req.outputName = output_name

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/compressFiles",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseCompressFilesT,
            flatbuffer_buf_size=2048,
            timeout=100,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Files Compress failed: obj_payload is None")

        return result["obj_payload"]

    async def decompress_file(self, robot_model: str, source_path: str, output_path: str) -> ResponseDecompressFileT:
        """
        [File Decompress 압축 해제]
        - robot_model: 요청을 보낼 로봇 모델
        - source_path: 압축 해제할 파일 경로
        - output_path: 압축 해제 파일 경로
        - ResponseDecompressFileT 객체 반환
        """

        # 1) RequestDecompressFileT 객체 생성
        req = RequestDecompressFileT()
        req.sourcePath = source_path
        req.outputPath = output_path

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/decompressFile",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseDecompressFileT,
            flatbuffer_buf_size=2048,
            timeout=100,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call File Decompress failed: obj_payload is None")

        return result["obj_payload"]

    async def delete_file(self, robot_model: str, file_path: str) -> ResponseDeleteFileT:
        """
        [File Delete 파일 삭제]
        - robot_model: 요청을 보낼 로봇 모델
        - file_path: 삭제할 파일 경로
        - ResponseDeleteFileT 객체 반환
        """

        # 1) RequestDeleteFileT 객체 생성
        req = RequestDeleteFileT()
        req.filePath = file_path

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/file/deleteFile",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseDeleteFileT,
            flatbuffer_buf_size=2048,
            timeout=100,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call File Delete failed: obj_payload is None")

        return result["obj_payload"]
