from typing import TypedDict


class Request_Save_Tool_ParameterSchema(TypedDict):
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

class Request_Save_User_Frame_ParameterSchema(TypedDict):
    userf_no: int
    userf_name: str
    userf_x: float
    userf_y: float
    userf_z: float
    userf_rx: float
    userf_ry: float
    userf_rz: float

class Request_Set_User_Frame_6DofSchema(TypedDict):
    user_frame_num: int
    setting_option: int
    target_x: float
    target_y: float
    target_z: float
    target_rx: float
    target_ry: float
    target_rz: float

class Request_Set_User_Frame_3PointsSchema(TypedDict):
    user_frame_num: int
    setting_option: int
    order_option: int
    point_1_x: float
    point_1_y: float
    point_1_z: float
    point_2_x: float
    point_2_y: float
    point_2_z: float
    point_3_x: float
    point_3_y: float
    point_3_z: float

class Request_Save_Area_ParameterSchema(TypedDict):
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

class Request_Set_Out_Collision_ParaSchema(TypedDict):
    onoff: int
    react_mode: int
    threshold: float

class Request_Set_Self_Collision_ParaSchema(TypedDict):
    mode: int
    dist_int: float
    dist_ext: float

class Request_Set_User_Frame_SelectionSchema(TypedDict):
    target_user_frame_num: int

class Request_Set_Fixed_Speed_ModeSchema(TypedDict):
    target_move: int
    vel_option: int
    vel_parameter: float
    acc_option: int
    acc_parameter: float
