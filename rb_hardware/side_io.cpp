#define P_NAME  "SIDE_IO"

#include "side_io.h"
#include "common.h"


side_io::side_io(int ch)
{
    for(int i = 0; i < NO_OF_DIN; ++i){
        states.din_raw[i] = 0;
        states.din_filt[i] = 0;
    }
    for(int i = 0; i < NO_OF_DOUT; ++i){
        states.dout_raw[i] = 0;
        desired_Dout[i] = 0;
    }
    for(int i = 0; i < NO_OF_AIN; ++i){
        states.adc_raw[i] = 0;
    }
    for(int i = 0; i < NO_OF_AOUT; ++i){
        states.dac_raw[i] = 0;
        desired_Aout[i] = 0;
    }
    cans.CAN_CH = ch;
    cans.CAN_ID_CONTROL     = 0x200;
    cans.CAN_ID_INPUT       = 0x201;
    cans.CAN_ID_OUTPUT      = 0x202;
    cans.CAN_ID_CMD         = 0x205;
    cans.CAN_ID_GENERAL     = 0x206;

    infos.firmware_version  = 0;
    infos.connection_timer  = 200;
    infos.connection_flag   = 0;    
}

side_io::~side_io(){
    ;
}

void side_io::onCANMessage(int ch, int id, const unsigned char* data, int dlc) {
    // id 기반으로 본인의 메시지만 필터링하고 처리
    if(ch != cans.CAN_CH){
        return;
    }

    if(id == cans.CAN_ID_INPUT){
        infos.connection_timer = 0;
        infos.connection_flag = true;

        states.adc_raw[0] = ((double)data[0])/20.;
        states.adc_raw[1] = ((double)data[1])/20.;
        states.adc_raw[2] = ((double)data[2])/20.;
        states.adc_raw[3] = ((double)data[3])/20.;

        unsigned int temp_ch = 0;

        temp_ch = data[4];
        states.din_raw[0] = (temp_ch >> 0) & 0x01;
        states.din_raw[1] = (temp_ch >> 1) & 0x01;
        states.din_raw[2] = (temp_ch >> 2) & 0x01;
        states.din_raw[3] = (temp_ch >> 3) & 0x01;
        states.din_raw[4] = (temp_ch >> 4) & 0x01;
        states.din_raw[5] = (temp_ch >> 5) & 0x01;
        states.din_raw[6] = (temp_ch >> 6) & 0x01;
        states.din_raw[7] = (temp_ch >> 7) & 0x01;

        temp_ch = data[5];
        states.din_raw[8] = (temp_ch >> 0) & 0x01;
        states.din_raw[9] = (temp_ch >> 1) & 0x01;
        states.din_raw[10] = (temp_ch >> 2) & 0x01;
        states.din_raw[11] = (temp_ch >> 3) & 0x01;
        states.din_raw[12] = (temp_ch >> 4) & 0x01;
        states.din_raw[13] = (temp_ch >> 5) & 0x01;
        states.din_raw[14] = (temp_ch >> 6) & 0x01;
        states.din_raw[15] = (temp_ch >> 7) & 0x01;

        for(int i = 0; i< NO_OF_DIN; ++i){
            states.din_filt[i] = sig_filter_din[i].update(states.din_raw[i]);
        }

        temp_ch = data[6];
        states.dout_init_stat = (temp_ch >> 1) & 0x01;
        states.din_init_stat = (temp_ch >> 2) & 0x01;
        states.dac_init_stat = (temp_ch >> 3) & 0x01;
        states.adc_init_stat = (temp_ch >> 4) & 0x01;
    }else if(id == cans.CAN_ID_OUTPUT){
        states.dac_raw[0] = ((double)data[0])/20.;
        states.dac_raw[1] = ((double)data[1])/20.;
        states.dac_raw[2] = ((double)data[2])/20.;
        states.dac_raw[3] = ((double)data[3])/20.;

        unsigned int temp_ch = 0;

        temp_ch = data[4];
        states.dout_raw[0] = (temp_ch >> 0) & 0x01;
        states.dout_raw[1] = (temp_ch >> 1) & 0x01;
        states.dout_raw[2] = (temp_ch >> 2) & 0x01;
        states.dout_raw[3] = (temp_ch >> 3) & 0x01;
        states.dout_raw[4] = (temp_ch >> 4) & 0x01;
        states.dout_raw[5] = (temp_ch >> 5) & 0x01;
        states.dout_raw[6] = (temp_ch >> 6) & 0x01;
        states.dout_raw[7] = (temp_ch >> 7) & 0x01;

        temp_ch = data[5];
        states.dout_raw[8] = (temp_ch >> 0) & 0x01;
        states.dout_raw[9] = (temp_ch >> 1) & 0x01;
        states.dout_raw[10] = (temp_ch >> 2) & 0x01;
        states.dout_raw[11] = (temp_ch >> 3) & 0x01;
        states.dout_raw[12] = (temp_ch >> 4) & 0x01;
        states.dout_raw[13] = (temp_ch >> 5) & 0x01;
        states.dout_raw[14] = (temp_ch >> 6) & 0x01;
        states.dout_raw[15] = (temp_ch >> 7) & 0x01;

        current_output_state.id = cans.CAN_ID_CONTROL;
        current_output_state.channel = cans.CAN_CH;
        current_output_state.dlc = 6;
        for(int i = 0; i < 6; i++){
            current_output_state.data[i] = data[i];
        }
    }else if(id == cans.CAN_ID_GENERAL){
        generals.gen_type = data[0];
        generals.gen_data = data[1];
        for(int i = 0; i < 8; ++i){
            if(i < dlc){
                generals.gen_bytes[i] = data[i];
            }else{
                generals.gen_bytes[i] = 0;
            }
        }
        generals.gen_is_new = true;

        if(generals.gen_type){
            infos.firmware_version = generals.gen_data;
        }
    }
}

void side_io::Set_Version(int t_version){
    infos.firmware_version = t_version;
}

void side_io::Set_ClearGeneralData(){
    generals.gen_data = 0;
    generals.gen_type = 0;
    for(int i = 0; i < 8; ++i){
        generals.gen_bytes[i] = 0;
    }
    generals.gen_is_new = false;
}

sINFO side_io::Get_Info(){
    return infos;
}

sSTAT side_io::Get_State(){
    return states;
}

sGENS side_io::Get_General(){
    return generals;
}

bool side_io::Set_ConnectionTimerUp(unsigned char up_cnt){
    infos.connection_timer += up_cnt;
    if(infos.connection_timer >= 200){
        infos.connection_timer = 200;
        if(infos.connection_flag){
            rb_common::log_push(LogLevel::Warning, "SIDE DISCON", P_NAME);
        }
        infos.connection_flag = false;
    }

    return infos.connection_flag;
}

void side_io::Set_Dout(unsigned int p_no, int value){
    if(p_no >= NO_OF_DOUT)  return;
    if(value != 1)  value = 0;

    desired_Dout[p_no] = value;
}

int side_io::Get_Dout(unsigned int p_no){
    if(p_no >= NO_OF_DOUT)  return 0;
    return desired_Dout[p_no];
}

void side_io::Set_Aout(unsigned int p_no, float value){
    if(p_no >= NO_OF_AOUT)   return;
    value = rb_math::saturation_L_and_U(value, 0, 10);

    desired_Aout[p_no] = value;
}

void side_io::Set_Din_Filter_Count(unsigned int p_no, int t_count){
    if(t_count < 0 || p_no >= NO_OF_DIN){
        return;
    }
    
    sig_filter_din[t_count].setThreshold(t_count);
}

void side_io::Set_Dout_Pulse(unsigned int p_no, unsigned char direction, float t1, float t2, float t3){
    if(p_no >= NO_OF_DOUT)  return;
    pulse_Dout_T1[p_no] = fabs(t1);
    pulse_Dout_T2[p_no] = fabs(t2);
    pulse_Dout_T3[p_no] = fabs(t3);
    pulse_Dout_Direction[p_no] = direction;
    pulse_Dout_Timer[p_no] = 0.;
    pulse_Dout_OnOff[p_no] = true;
}
bool side_io::Get_Dout_Pulse_State(unsigned int p_no){
    if(p_no >= NO_OF_DOUT)  return false;
    return pulse_Dout_OnOff[p_no];
}
void side_io::Update_Dout_Pulse(float dt){
    for(int k = 0; k < NO_OF_DOUT; ++k){
        if(pulse_Dout_OnOff[k]){
            if(pulse_Dout_Timer[k] < pulse_Dout_T1[k]){
                Set_Dout(k, (pulse_Dout_Direction[k]));
            }else if(pulse_Dout_Timer[k] < pulse_Dout_T2[k]){
                Set_Dout(k, (!pulse_Dout_Direction[k]));
            }else if(pulse_Dout_Timer[k] < pulse_Dout_T3[k]){
                Set_Dout(k, (pulse_Dout_Direction[k]));
            }else{
                pulse_Dout_OnOff[k] = false;
            }
            pulse_Dout_Timer[k] += dt;
        }else{
            pulse_Dout_Timer[k] = 0.;
        }
    }
}

CAN_MSG side_io::CmdRequestVersion(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 1;
    can_m.dlc = 1;
    return can_m;
}

CAN_MSG side_io::CmdIOControl(){
    unsigned int temp_byte_4 = 0;
    unsigned int temp_byte_5 = 0;
    for(int i = 0; i < 8; ++i){
        temp_byte_4 |= (desired_Dout[i + 0] << i);
        temp_byte_5 |= (desired_Dout[i + 8] << i);
    }
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CONTROL;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = (unsigned char)(desired_Aout[0] * 20.);
    can_m.data[1] = (unsigned char)(desired_Aout[1] * 20.);
    can_m.data[2] = (unsigned char)(desired_Aout[2] * 20.);
    can_m.data[3] = (unsigned char)(desired_Aout[3] * 20.);
    can_m.data[4] = temp_byte_4 & 0xFF;
    can_m.data[5] = temp_byte_5 & 0xFF;
    can_m.dlc = 6;
    return can_m;
}

CAN_MSG side_io::GetCurrentOutputState(){
    return current_output_state;
}