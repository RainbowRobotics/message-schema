from typing import Literal, TypedDict


class DigitalInputConditionSchema(TypedDict):
    """Wait 커맨드에 사용되는 Digital Input 조건"""
    signal: list[int]
    logical_operator: Literal["AND", "OR"]

class ModbusClientWritePayloadSchema(TypedDict):
    """Modbus Client 호출 인자"""
    issue_core: str
    server_ip: str
    server_port: int
    function_code: int
    register_addr: int
    payload_dlc: int
    payload: list[int]
    timeout_ms: int

class ModbusClientReadPayloadSchema(TypedDict):
    """Modbus Client 호출 인자"""
    issue_core: str
    server_ip: str
    server_port: int
    function_code: int
    register_addr: int
    number_of_read: int
    timeout_ms: int

class InterfaceModbusClientWriteOptionSchema(TypedDict):
    """Interface 호출 인자"""
    register_addr: int
    payload_dlc: int
    payload: list[int]

class InterfaceModbusClientReadOptionSchema(TypedDict):
    """Interface 호출 인자"""
    register_addr: int
    number_of_read: int
    timeout_ms: int

class InterfaceModbusClientPayloadSchema(TypedDict):
    """Interface 호출 인자"""
    type: Literal["READ", "WRITE"]
    server_ip: str
    server_port: int
    function_code: int
    register_addr: int
    timeout_ms: int
    payload_dlc: int | None
    payload: list[int] | None
    number_of_read: int | None
    return_variable_name: str | None
