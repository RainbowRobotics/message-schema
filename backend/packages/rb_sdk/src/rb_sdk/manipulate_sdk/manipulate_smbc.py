""" Rainbow Robotics Manipulate SMBC SDK """
from rb_flat_buffers.SMBC.N_32_i import N_32_iT
from rb_flat_buffers.SMBC.Request_Read import Request_ReadT
from rb_flat_buffers.SMBC.Request_Write import Request_WriteT
from rb_flat_buffers.SMBC.Response_Read import Response_ReadT
from rb_flat_buffers.SMBC.Response_Write import Response_WriteT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK


class RBManipulateSMBCSdk(RBBaseSDK):
    """Rainbow Robotics Manipulate SMBC SDK"""

    def __init__(self):
        super().__init__()

    def modbus_read(
            self,
            *,
            issue_core: str,
            server_ip: str,
            server_port: int,
            function_code: int,
            register_addr: int,
            number_of_read: int,
            timeout_ms: int | None = 0,
            flow_manager_args: FlowManagerArgs | None = None
        ):
        """
        [SMBC 호출 함수]

        Args:
            issue_core: 이슈 코어
            server_ip: 서버 IP
            server_port: 서버 포트
            function_code: 함수 코드
            register_addr: 레지스터 주소
            number_of_read: 읽을 데이터 개수
            timeout_ms: 타임아웃 시간
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)

        Returns:
            Response_ReadT: 읽은 데이터
        """

        req = Request_ReadT()
        req.issueCore = issue_core
        req.serverIp = server_ip
        req.serverPort = server_port
        req.functionCode = function_code
        req.registerAddr = register_addr
        req.numberOfRead = number_of_read
        req.timeoutMs = timeout_ms

        topic = "SMBC/read_word" if function_code == 3 or function_code == 4 else "SMBC/read_bit"

        res = self.zenoh_client.query_one(
            topic,
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_ReadT,
            flatbuffer_buf_size=40,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Modbus Read failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]


    def modbus_write(
            self,
            *,
            issue_core: str,
            server_ip: str,
            server_port: int,
            function_code: int,
            register_addr: int,
            payload_dlc: int, payload: list[int],
            timeout_ms: int | None = 0,
            flow_manager_args: FlowManagerArgs | None = None,
        ):
        """
        [SMBC 호출 함수]

        Args:
            issue_core: 이슈 코어
            server_ip: 서버 IP
            server_port: 서버 포트
            function_code: 함수 코드
            register_addr: 레지스터 주소
            payload_dlc: 페이로드 데이터 개수
            payload: 페이로드 데이터
            timeout_ms: 타임아웃 시간
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)

        Returns:
            Response_WriteT: 쓴 데이터
        """

        if not isinstance(payload, list):
            raise ValueError("payload must be a list")

        if payload_dlc > len(payload):
            raise ValueError("payload_dlc is greater than payload length")

        if len(payload) > 32:
            raise ValueError("payload length cannot exceed 32")

        req = Request_WriteT()
        req.issueCore = issue_core
        req.serverIp = server_ip
        req.serverPort = server_port
        req.functionCode = function_code
        req.registerAddr = register_addr
        req.payloadDlc = 1 if function_code == 6 or function_code == 5 else payload_dlc
        req.timeoutMs = timeout_ms

        padded_payload = payload + [0] * (32 - len(payload))

        ni = N_32_iT()
        ni.i = padded_payload
        req.payload = ni

        topic = "SMBC/write_word" if function_code == 6 or function_code == 16 else "SMBC/write_bit"

        res = self.zenoh_client.query_one(
            topic,
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_WriteT,
            flatbuffer_buf_size=40,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Modbus Write failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]
