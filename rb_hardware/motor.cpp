#define P_NAME  "MOTOR"

#include <string>

#include "motor.h"
#include "common.h"
#include "rmath.h"


#define ENCODER_RCV_HZ  250

motor::motor(int bno, int ch)
    : mA_filter(ENCODER_RCV_HZ, ENCODER_RCV_HZ * 60)
{
    cans.CAN_BNO = bno;
    cans.CAN_CH = ch;
    cans.CAN_ID_BOOT = 900 + bno;
    cans.CAN_ID_CMD = 0x10 + bno;
    cans.CAN_ID_PAR = 0x20 + bno;
    cans.CAN_ID_STA = 0x30 + bno;
    cans.CAN_ID_REF = 0x40 + bno;
    cans.CAN_ID_ENC = 0x50 + bno;

    infos.connection_timer = 200;
    infos.connection_flag = 0;

    Clear_States();
    Clear_Infos(0);

    vel_lpf_alpha = rb_math::Calc_LPF_Alpha(((double)ENCODER_RCV_HZ), 4.5);
    acc_lpf_alpha = rb_math::Calc_LPF_Alpha(((double)ENCODER_RCV_HZ), 4.5);
}

motor::~motor(){
    ;
}

void motor::onCANMessage(int ch, int id, const unsigned char* data, int dlc) {
    if(ch == cans.CAN_CH){
        if(id == cans.CAN_ID_BOOT){
            unsigned int raw_1 = data[0];
            unsigned int raw_2 = data[1];
            unsigned int uvw_err = raw_1 & 0b1111;
            // 0 = no err
            // 0 = u
            // 1 = v
            // 2 = w
            // 4 = x
            unsigned int curr_err = (raw_1>>4) & 0b1111;
            // 0 = u sang cur
            // 1 = v sang cur
            // 2 = w sang cur
            unsigned int multi_err = raw_2 & 0b1;
            unsigned int enc_matching_cnt = (raw_2 >> 1) & 0b11;
            unsigned int dq__matching_cnt = (raw_2 >> 3) & 0b11;
            // 0 = no problem
            // 1 = 120'
            infos.stat_uvw = uvw_err;
            infos.stat_cur = curr_err;
            infos.stat_mul = multi_err;
            infos.stat_ram_enc = enc_matching_cnt;
            infos.stat_ram_dqp = dq__matching_cnt;
            int temp_enc = (int)((data[2]) | (data[3]<<8) | (data[4]<<16) | (data[5]<<24));
            infos.stat_last_enc = ((double)temp_enc) * parameters.para_pulse_to_deg;

            //std::cout<<"MOTOR "<< CAN_BNO <<" info: /UVW/ "<<((int)into_stat_uvw)<<" /CUR/ "<<((int)into_stat_cur)<<" /MUL/ "<<((int)into_stat_mul)<<" /RAM1/ "<<((int)into_stat_ram_enc)<<" /RAM2/ "<<((int)into_stat_ram_dqp)<<" /LP/ "<<((float)info_stat_last_enc)<<std::endl;
            
            if((data[6] & 0b11) == 3 || (data[6] & 0b11) == 2){
                unsigned char U_sense_Err = (data[6]>>4) & 0b01;
                unsigned char V_sense_Err = (data[6]>>3) & 0b01;
                unsigned char W_sense_Err = (data[6]>>2) & 0b01;
                //std::cout<<"Current Sensor Error: "<<(int)U_sense_Err<<" / "<<(int)V_sense_Err<<" / "<<(int)W_sense_Err<<std::endl;

                infos.stat_sensors = 1 + U_sense_Err * 2 + V_sense_Err * 4 + W_sense_Err * 8;

                if((data[6] & 0b11) == 3){
                    infos.type_blm = 0;
                }else{
                    infos.type_blm = 1;
                }
            }else{
                infos.stat_sensors = 0;
                infos.type_blm = 0;
            }

            if(dlc == 8){
                if(((data[6]>>5) & 0b01) == 1){
                    int d6 = data[6];
                    int d7 = data[7];
                    infos.type_num = (((d6>>7) & 0b01) << 8) | (d7 & 0xFF);
                    infos.type_mdr = (d6>>6) & 0b01;
                }else{
                    infos.type_num = -1;
                    infos.type_mdr = -1;
                }
            }else{
                infos.type_num = -1;
                infos.type_mdr = -1;
            }

            std::string msg = "BNO "  + std::to_string(cans.CAN_BNO) 
                            + " TYP:" + std::to_string(infos.type_num)
                            + " MDR:" + std::to_string(infos.type_mdr)
                            + " BLM:" + std::to_string(infos.type_blm)
                            + " SEN:" + std::to_string(infos.stat_sensors)
                            + " SLE:" + std::to_string(infos.stat_last_enc);
            rb_common::log_push(LogLevel::Info, msg, P_NAME);
        }else if(id == cans.CAN_ID_ENC){
            infos.connection_timer = 0;
            infos.connection_flag = true;

            int temp_enc = (int)((data[0]) | (data[1]<<8) | (data[2]<<16));
            if(temp_enc & 0x800000){
                temp_enc |= 0xFF000000;
            }
            int temp_cur = (int)((short)(data[3] | (data[4]<<8)));
            infos.encoder_pulse = temp_enc;
            infos.encoder_deg = ((double)temp_enc) * parameters.para_pulse_to_deg;

            infos.encoder_deg_vel = (infos.encoder_deg - infos.encoder_deg_prev) * ((double)ENCODER_RCV_HZ);
            infos.encoder_deg_vel_LPF = vel_lpf_alpha * infos.encoder_deg_vel_LPF + (1.0 - vel_lpf_alpha) * infos.encoder_deg_vel;

            infos.encoder_deg_acc = (infos.encoder_deg_vel_LPF - infos.encoder_deg_vel_LPF_prev) * ((double)ENCODER_RCV_HZ);
            infos.encoder_deg_acc_LPF = acc_lpf_alpha * infos.encoder_deg_acc_LPF + (1.0 - acc_lpf_alpha) * infos.encoder_deg_acc;
            infos.encoder_deg_vel_LPF_prev = infos.encoder_deg_vel_LPF;
            infos.encoder_deg_prev = infos.encoder_deg;
            infos.encoder_deg_error = last_ref_angle_deg - infos.encoder_deg;

            Activation_Process_Update(infos.encoder_deg);

            infos.torque_mA = temp_cur;
            infos.torque_Nm = temp_cur * 0.001 * parameters.para_torque_const * parameters.para_reduction_rate;

            rb_math::MovingAverage::Result t_filt = mA_filter.filter(infos.torque_mA);
            infos.torque_mA_movingFiltered = t_filt.avg_full;
            infos.torque_Nm_movingFiltered = infos.torque_mA_movingFiltered * 0.001 * parameters.para_torque_const * parameters.para_reduction_rate;

            states.b.RUN = (data[5] >> 0) & 0x01;//
            states.b.MOD = (data[5] >> 1) & 0x01;//
            states.b.JAM = (data[5] >> 2) & 0x01;//
            states.b.CUR = (data[5] >> 3) & 0x01;//
            states.b.BIG = (data[5] >> 4) & 0x01;//
            states.b.INP = (data[5] >> 5) & 0x01;//
            states.b.PS1 = 0;
            states.b.PS2 = 0;
            states.b.MT_ERR = 0;
            states.b.EST_TMP = (data[5] >> 6) & 0x01;
            states.b.CUR_SUM = (data[5] >> 7) & 0x01;

            if(((data[6]>>7)&0x01) == 0){
                infos.temperature_board = data[6];
            }else{
                infos.temperature_motor = (int)(data[6]&0x7F);
            }
        }else if(id == cans.CAN_ID_PAR){
            if(data[0] == 0x01){
                infos.firmware_version = data[3] | (data[4]<<8) | (data[5]<<16) | (data[6]<<24);
            }else if(data[0] == 0xD2){
                infos.gain_current_P = (short)(data[1] | (data[2]<<8));
                infos.gain_current_I = (short)(data[3] | (data[4]<<8));
                infos.gain_current_flag = true;
            }else if(data[0] == 0xD5){
                infos.gain_position_P = (short)(data[1] | (data[2]<<8));
                infos.gain_position_I = (short)(data[3] | (data[4]<<8));
                infos.gain_position_D = (short)(data[5] | (data[6]<<8));
                infos.gain_position_flag = true;
            }
        }else if(id == cans.CAN_ID_STA){
            states.B[0] = data[0];
            states.B[1] = data[1];
            states.B[2] = data[2];
        }
    }else{
        if(id == cans.CAN_ID_BOOT){
            std::string msg = "BNO "  + std::to_string(cans.CAN_BNO) + "is detected in wrong channel";
            rb_common::log_push(LogLevel::Error, msg, P_NAME);
        }
    }
}

void motor::Clear_States(){
    states.B[0] = states.B[1] = states.B[2] = states.B[3] = 0;
}

mSTAT motor::Get_States(){
    return states;
}

void motor::Clear_Infos(int mode){
    // mode
    // 0 for class init
    // 1 for power off

    
    infos.encoder_deg_vel_LPF = 0;
    infos.encoder_deg_vel_LPF_prev = 0;
    infos.encoder_deg_acc = 0;
    infos.encoder_deg_acc_LPF = 0;

    infos.gain_current_flag = 0;
    infos.gain_position_flag = false;

    if(mode == 0){
        infos.encoder_deg = 0;
        infos.encoder_deg_prev = 0;
        infos.encoder_deg_error = 0;
        infos.encoder_pulse = 0;
        infos.temperature_motor = infos.temperature_board = 0;
    }

    mA_filter.reset();    
    infos.torque_mA = infos.torque_Nm = infos.torque_mA_movingFiltered = infos.torque_Nm_movingFiltered = 0;    
}

mINFO motor::Get_Infos(){
    return infos;
}

void motor::Set_Parameters(mPARA tPara){
    parameters = tPara;
}

mPARA motor::Get_Parameters(){
    return parameters;
}

void motor::Set_Version(int version){
    infos.firmware_version = version;
}

bool motor::Set_ConnectionTimerUp(unsigned char up_cnt){
    infos.connection_timer += up_cnt;
    if(infos.connection_timer >= 200){
        infos.connection_timer = 200;
        if(infos.connection_flag){
            rb_common::log_push(LogLevel::Warning, "MOTOR DISCON " + std::to_string(cans.CAN_BNO), P_NAME);
        }
        infos.connection_flag = false;
    }
    return infos.connection_flag;
}

bool motor::Get_ConnectionFlag(){
    return infos.connection_flag;
}

void motor::Activation_Process_Start(){
    activating_start_angle = infos.encoder_deg;
    activating_min_angle = 0;
    activating_max_angle = 0;
    activating_flag = true;
}

void motor::Activation_Process_Update(double current_angle){
    if(activating_flag){
        activating_min_angle = rb_math::return_small(activating_min_angle, (current_angle - activating_start_angle));
        activating_max_angle =   rb_math::return_big(activating_max_angle, (current_angle - activating_start_angle));
    }
}

std::tuple<double, double> motor::Activation_Process_Stop(){
    activating_flag = false;
    return {activating_min_angle, activating_max_angle};
}

void motor::Set_Last_Reference(double angle_deg, double torque_Nm, double fb_gain, double ff_gain, int tq_limit_A){
    last_ref_angle_deg = angle_deg;
    last_ref_torque_Nm = torque_Nm;
    last_ref_fb_gain = fb_gain;
    last_ref_ff_gain = ff_gain;
    last_ref_tq_limit_A = tq_limit_A;

    infos.encoder_deg_error = last_ref_angle_deg - infos.encoder_deg;
}

CAN_MSG motor::CmdAdminMode(unsigned int onoff){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xDC;
    can_m.data[1] = 0xAA;
    can_m.data[2] = 0xBB;
    can_m.data[3] = 0xCC;
    can_m.data[4] = (onoff & 0b01);
    can_m.dlc = 5;
    return can_m;
}

CAN_MSG motor::CmdBlindError(unsigned char blind_big, unsigned char blind_inp){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xDD;
    can_m.data[1] = 37;//password
    can_m.data[2] = (blind_big & 0b01);
    can_m.data[3] = (blind_inp & 0b01);
    can_m.dlc = 4;
    return can_m;
}

void motor::Clear_Gain_Current(){
    infos.gain_current_flag = false;
}
CAN_MSG motor::Cmd_Ask_Gain_Current(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xD2;
    can_m.data[1] = 1;
    can_m.dlc = 2;
    return can_m;
}
CAN_MSG motor::Cmd_Save_Gain_Current(unsigned int gain_P, unsigned int gain_I){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xD2;
    can_m.data[1] = 0;
    can_m.data[2] = (gain_P >> 0) & 0xFF;
    can_m.data[3] = (gain_P >> 8) & 0xFF;
    can_m.data[4] = (gain_I >> 0) & 0xFF;
    can_m.data[5] = (gain_I >> 8) & 0xFF;
    can_m.dlc = 6;
    return can_m;
}
void motor::Clear_Gain_Position(){
    infos.gain_position_flag = false;
}
CAN_MSG motor::Cmd_Temporary_Gain_Position(unsigned char set_mode, unsigned int gain_P, unsigned int gain_I, unsigned int gain_D){
    if(set_mode == 1){
        CAN_MSG can_m;
        can_m.id = cans.CAN_ID_CMD;
        can_m.channel = cans.CAN_CH;
        can_m.data[0] = 0xDA;
        can_m.data[1] = 1;//set temporary gain
        can_m.data[2] = (gain_P >> 0) & 0xFF;
        can_m.data[3] = (gain_P >> 8) & 0xFF;
        can_m.data[4] = (gain_I >> 0) & 0xFF;
        can_m.data[5] = (gain_I >> 8) & 0xFF;
        can_m.data[6] = (gain_D >> 0) & 0xFF;
        can_m.data[7] = (gain_D >> 8) & 0xFF;
        can_m.dlc = 8;
        return can_m;
    }else{
        CAN_MSG can_m;
        can_m.id = cans.CAN_ID_CMD;
        can_m.channel = cans.CAN_CH;
        can_m.data[0] = 0xDA;
        can_m.data[1] = 0;//goto default
        can_m.dlc = 2;
        return can_m;
    }
}
CAN_MSG motor::Cmd_Ask_Gain_Position(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xD5;
    can_m.data[1] = 1;
    can_m.dlc = 2;
    return can_m;
}
CAN_MSG motor::Cmd_Save_Gain_Posision(unsigned int gain_P, unsigned int gain_I, unsigned int gain_D){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xD5;
    can_m.data[1] = 0;
    can_m.data[2] = (gain_P >> 0) & 0xFF;
    can_m.data[3] = (gain_P >> 8) & 0xFF;
    can_m.data[4] = (gain_I >> 0) & 0xFF;
    can_m.data[5] = (gain_I >> 8) & 0xFF;
    can_m.data[6] = (gain_D >> 0) & 0xFF;
    can_m.data[7] = (gain_D >> 8) & 0xFF;
    can_m.dlc = 8;
    return can_m;
}

CAN_MSG motor::CmdServoOn(double esti_torque_Nm){

    int esti_mA = (esti_torque_Nm / parameters.para_reduction_rate / parameters.para_torque_const * 1000.);

    if(esti_mA > 32767){
        esti_mA = +32767;
    }else if(esti_mA < -32767){
        esti_mA = -32767;
    }

    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xC3;
    can_m.data[1] = 0;
    can_m.data[2] = esti_mA & 0xFF;
    can_m.data[3] = (esti_mA >> 8) & 0xFF;
    can_m.dlc = 4;

    return can_m;
}

CAN_MSG motor::CmdRequestVersion(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0x01;
    can_m.dlc = 1;
    return can_m;
}

CAN_MSG motor::CmdRequestStatus(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0x02;
    can_m.dlc = 1;
    return can_m;
}

CAN_MSG motor::CmdControl(){
    return CmdControl(last_ref_angle_deg, last_ref_torque_Nm, last_ref_fb_gain, last_ref_ff_gain, last_ref_tq_limit_A);
}

CAN_MSG motor::CmdControl(double position_deg, double torque_Nm, double fb_gain, double ff_gain, int tq_limit_A){
    position_deg = rb_math::saturation_Low(position_deg, parameters.para_limit_angleDeg_Low);
    position_deg = rb_math::saturation_Up(position_deg, parameters.para_limit_angleDeg_Up);
    torque_Nm = rb_math::saturation_Up(torque_Nm, parameters.para_limit_torqueNm);
    torque_Nm = rb_math::saturation_Low(torque_Nm, -parameters.para_limit_torqueNm);
    fb_gain = rb_math::saturation_L_and_U(fb_gain, 0, 1.5);
    ff_gain = rb_math::saturation_L_and_U(ff_gain, 0, 1.5);
    tq_limit_A = rb_math::saturation_Up(tq_limit_A, 63);

    Set_Last_Reference(position_deg, torque_Nm, fb_gain, ff_gain, tq_limit_A);

    int t_position_pulse = position_deg / parameters.para_pulse_to_deg;
    int t_send_pos_digit = t_position_pulse & 0x007FFFFF;
    if(t_position_pulse < 0){
        t_send_pos_digit |= 0x00800000;
    }
    int t_current_mA = (torque_Nm / parameters.para_reduction_rate / parameters.para_torque_const * 1000.);
    if(t_current_mA > +32767){
        t_current_mA = +32767;
    }else if(t_current_mA < -32767){
        t_current_mA = -32767;
    }
    short t_send_cur_digit = t_current_mA;

    int t_fb_gain = round(10.0 * fb_gain);
    int t_ff_gain = round(10.0 * ff_gain);

    int t_enc_request_T = 0;

    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_REF;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = t_send_pos_digit & 0xFF;
    can_m.data[1] = (t_send_pos_digit>>8) & 0xFF;
    can_m.data[2] = (t_send_pos_digit>>16) & 0xFF;
    can_m.data[3] = (t_send_cur_digit & 0xFF);
    can_m.data[4] = (t_send_cur_digit>>8) & 0xFF;
    can_m.data[5] = (unsigned char)(t_enc_request_T & 0b10111111);
    // if(MY_CONTROL_MODE_SELECTION == MY_CONTROL_MODE_CUR){
    //     can_m.data[5] = can_m.data[5] | 0b01000000;
    // }
    can_m.data[6] = ((t_fb_gain<<4) & 0xF0) | (t_ff_gain & 0x0F);
    can_m.dlc = 7;

    if(tq_limit_A >= 0){
        can_m.data[7] = 0b01000000 | (tq_limit_A & 0b00111111);
        can_m.dlc = 8;
    }
    return can_m;
}

CAN_MSG motor::CmdMakeErrorSumZero(unsigned char mode){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0xDD;
    can_m.data[1] = 47;
    if(mode == 1){
        can_m.data[1] = 48;
    }
    can_m.dlc = 2;
    return can_m;
}