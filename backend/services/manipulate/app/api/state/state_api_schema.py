from pydantic import BaseModel


class PowerControlRequestPD(BaseModel):
    power_option: int
    sync_servo: bool | None = True


class ServoControlRequestPD(BaseModel):
    servo_option: int


class ReferenceControlRequestPD(BaseModel):
    reference_option: int


class StateRequestPD(BaseModel):
    joint_q_ref: list[float]
    joint_q_enc: list[float]
    joint_t_esti: list[float]
    joint_t_meas: list[float]
    joint_temper: list[float]

    carte_x_ref: list[float]
    carte_x_enc: list[float]

    cbox_digital_input: list[int]
    cbox_digital_output: list[int]

    motion_mode: int
    motion_speed_bar: float
    motion_is_pause: int

    status_lan2can: int
    status_switch_emg: int
    status_power_out: int
    status_servo_num: int
    status_is_refon: int
    status_out_coll: int
    status_self_coll: int
    status_dt_mode: int
