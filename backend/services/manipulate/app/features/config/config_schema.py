from pydantic import BaseModel
from rb_schemas.base import Vec3fPD


class ST_Box_ParaPD(BaseModel):
    f: list[float]


class ST_Config_UserFramePD(BaseModel):
    userfName: str
    userfOffset: Vec3fPD
    userfEuler: Vec3fPD


class ST_Tool_ParaPD(BaseModel):
    toolName: str
    comMass: float
    comOffset: list[float]
    tcpOffset: list[float]
    tcpEuler: list[float]
    boxType: int
    boxParameter: ST_Box_ParaPD


class ST_Config_AreaPD(BaseModel):
    areaName: str
    areaType: int
    areaOffset: list[float]
    areaEuler: list[float]
    areaPara: list[float]


class Response_CallConfigRobotArmPD(BaseModel):
    outCollOnoff: int
    outCollReact: int
    outCollLimit: float
    directTeachingSensitivity: list[float]


class Response_CallConfigToolListPD(BaseModel):
    tConfigs0: ST_Tool_ParaPD
    tConfigs1: ST_Tool_ParaPD
    tConfigs2: ST_Tool_ParaPD
    tConfigs3: ST_Tool_ParaPD
    tConfigs4: ST_Tool_ParaPD
    tConfigs5: ST_Tool_ParaPD
    tConfigs6: ST_Tool_ParaPD
    tConfigs7: ST_Tool_ParaPD


class Response_CallConfigControlBoxPD(BaseModel):
    doutSpecialFunc: list[int]
    dinSpecialFunc: list[int]
    dinFilterCount: list[int]
    areaConfigs0: ST_Config_AreaPD
    areaConfigs1: ST_Config_AreaPD
    areaConfigs2: ST_Config_AreaPD
    areaConfigs3: ST_Config_AreaPD
    areaConfigs4: ST_Config_AreaPD
    areaConfigs5: ST_Config_AreaPD
    areaConfigs6: ST_Config_AreaPD
    areaConfigs7: ST_Config_AreaPD
    userFrame0: ST_Config_UserFramePD
    userFrame1: ST_Config_UserFramePD
    userFrame2: ST_Config_UserFramePD
    userFrame3: ST_Config_UserFramePD
    userFrame4: ST_Config_UserFramePD
    userFrame5: ST_Config_UserFramePD
    userFrame6: ST_Config_UserFramePD
    userFrame7: ST_Config_UserFramePD


class Request_Save_Area_ParameterPD(BaseModel):
    area_no: int
    area_name: str
    area_type: int
    area_x: float
    area_y: float
    area_z: float
    area_rx: float
    area_ry: float
    area_rz: float
    area_para_0: float
    area_para_1: float
    area_para_2: float


class Request_Save_Tool_List_ParameterPD(BaseModel):
    tool_no: int
    tool_name: str
    tcp_x: float
    tcp_y: float
    tcp_z: float
    tcp_rx: float
    tcp_ry: float
    tcp_rz: float
    mass_m: float
    mass_x: float
    mass_y: float
    mass_z: float
    box_type: int
    box_para_0: float
    box_para_1: float
    box_para_2: float
    box_para_3: float
    box_para_4: float
    box_para_5: float
    box_para_6: float
    box_para_7: float
    box_para_8: float


class Request_Save_Direct_Teach_SensitivityPD(BaseModel):
    sensitivity: list[float]


class Request_Save_SideDin_FilterPD(BaseModel):
    port_num: int
    desired_count: int


class Request_Save_SideDin_FunctionPD(BaseModel):
    port_num: int
    desired_function: int


class Request_Save_SideDout_FunctionPD(BaseModel):
    port_num: int
    desired_function: int


class Request_Save_Collision_ParameterPD(BaseModel):
    onoff: int
    react: int
    threshold: float


class Request_Save_SelfColl_ParameterPD(BaseModel):
    mode: int
    dist_internal: float
    dist_external: float


class Request_Set_Tool_ListPD(BaseModel):
    target_tool_num: int


class Request_Save_User_FramePD(BaseModel):
    userf_no: int
    userf_name: str
    userf_x: float
    userf_y: float
    userf_z: float
    userf_rx: float
    userf_ry: float
    userf_rz: float


class Response_UserFrameParameterPD(BaseModel):
    user_frames: list[ST_Config_UserFramePD]
