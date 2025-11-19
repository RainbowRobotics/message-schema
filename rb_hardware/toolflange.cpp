#define P_NAME  "TOOLFLANGE"

#include "toolflange.h"
#include "common.h"

toolflange::toolflange(int ch)
{
    cans.CAN_CH = ch;
    cans.CAN_ID_CMD = 0x100;
    cans.CAN_ID_GMBUS_SET = 0x102;
    cans.CAN_ID_GMBUS_WRITE = 0x103;

    cans.CAN_ID_GYR = 0x110;
    cans.CAN_ID_ACC = 0x111;
    cans.CAN_ID_485 = 0x112;
    cans.CAN_ID_MBS_LEGACY_RX = 0x113;
    cans.CAN_ID_GMBUS_READ_SINGLE = 0x114;
    cans.CAN_ID_PAR = 0x115;
    cans.CAN_ID_GEN = 0x120;
    cans.CAN_ID_BOOT = 899;

    states.button = 0;
    states.voltage = 0;
    states.temperature = 0;
    for(int i = 0; i < TFB_NUM_DIN; i++){
        states.din[i] = 0;
    }
    for(int i = 0; i < TFB_NUM_DOUT; i++){
        states.dout[i] = 0;
    }

    infos.firmware_version = 0;
    infos.connection_timer = 200;
    infos.connection_flag = 0;
}

toolflange::~toolflange(){
    ;
}

void toolflange::onCANMessage(int ch, int id, const unsigned char* data, int dlc) {
    // id 기반으로 본인의 메시지만 필터링하고 처리
    if(ch != cans.CAN_CH){
        return;
    }

    if(id == cans.CAN_ID_GYR){
        states.gyro[0] = ((double)((short)(data[0] | (data[1] << 8))))/65.;
        states.gyro[1] = ((double)((short)(data[2] | (data[3] << 8))))/65.;
        states.gyro[2] = ((double)((short)(data[4] | (data[5] << 8))))/65.;

        int temp_dat = data[6];
        states.din[2] = (temp_dat >> 7) & 0b01;
        states.din[3] = (temp_dat >> 6) & 0b01;
        states.din[4] = (temp_dat >> 5) & 0b01;
        states.din[3] = (temp_dat >> 4) & 0b01;
    }else if(id == cans.CAN_ID_ACC){
        infos.connection_timer = 0;
        infos.connection_flag = true;

        states.acc[0] = ((float)((short)(data[0] | (data[1] << 8))))/1000.;
        states.acc[1] = ((float)((short)(data[2] | (data[3] << 8))))/1000.;
        states.acc[2] = ((float)((short)(data[4] | (data[5] << 8))))/1000.;

        states.button       = ((data[6] >> 7) & 0x01);
        states.temperature  = (data[6] & 0b01111111);
        states.din[0]       = ((data[7] >> 7) & 0x01);
        states.din[1]       = ((data[7] >> 6) & 0x01);
        states.dout[0]      = ((data[7] >> 5) & 0x01);
        states.dout[1]      = ((data[7] >> 4) & 0x01);
        states.rs485        = ((data[7] >> 3) & 0x01);
        states.voltage      = ((data[7] >> 0) & 0b11)*12;
    }else if(id == cans.CAN_ID_485){
        ;
    }else if(id == cans.CAN_ID_MBS_LEGACY_RX){
        if(data[0] == 15){
            if(dlc == 6){
                unsigned int r_id     = data[1];
                unsigned int r_addr   = data[2] | (data[3] << 8);
                unsigned int r_data_0 = data[4] | (data[5] << 8);
                std::cout<<"GMbus Read Multi Type1: "<<r_id<<" / "<<r_addr<<" / "<<r_data_0<<std::endl;
            }else if(dlc == 8){
                unsigned int r_id     = data[1];
                unsigned int r_addr   = data[2] | (data[3] << 8);
                unsigned int r_data_0 = data[4] | (data[5] << 8);
                unsigned int r_data_1 = data[6] | (data[7] << 8);
                std::cout<<"GMbus Read Multi Type2: "<<r_id<<" / "<<r_addr<<" / "<<r_data_0<<" / "<<r_data_1<<std::endl;
            }
        }
    }else if(id == cans.CAN_ID_GMBUS_READ_SINGLE){
        if(dlc == 6){
            unsigned int r_id     = data[1];
            unsigned int r_addr   = data[2] | (data[3] << 8);
            unsigned int r_data_0 = data[4] | (data[5] << 8);
            std::cout<<"GMbus Read Single Type1: "<<r_id<<" / "<<r_addr<<" / "<<r_data_0<<std::endl;
        }else if(dlc == 8){
            unsigned int r_id     = data[1];
            unsigned int r_addr   = data[2] | (data[3] << 8);
            unsigned int r_data_0 = data[4] | (data[5] << 8);
            unsigned int r_data_1 = data[6] | (data[7] << 8);
            std::cout<<"GMbus Read Single Type2: "<<r_id<<" / "<<r_addr<<" / "<<r_data_0<<" / "<<r_data_1<<std::endl;
        }
    }else if(id == cans.CAN_ID_PAR){
        ;
    }else if(id == cans.CAN_ID_GEN){
        generals.gen_type = (unsigned int)data[0];
        generals.gen_data = (unsigned int)(data[1] | (data[2]<<8) | (data[3]<<16) | (data[4]<<24));
        if(dlc >= 6){
            generals.gen_sub_type = (unsigned int)data[5];
        }else{
            generals.gen_sub_type = 0;
        }
        generals.gen_is_new = true;

        if(generals.gen_type == 1){
            infos.firmware_version = generals.gen_data;
        }
    }else if(id == cans.CAN_ID_BOOT){
        if(dlc >= 4){
            int tfb_version = data[0];
            int tfb_can_spd = data[1];
            int tfb_silicon = data[2];
            int tfb_calib_s = data[3];
            int tfb_imu_info = -1;

            if(dlc >= 5){
                tfb_imu_info = data[4];
            }

            char temp_tfb_boot[50];
            snprintf(temp_tfb_boot, 50, "TFB RST %d %d %d %d %d", tfb_version, tfb_can_spd, tfb_silicon, tfb_calib_s, tfb_imu_info);

            std::string temp_tfb_str(temp_tfb_boot);
            LOG_INFO(temp_tfb_str);
        }
    }
}

tINFO toolflange::Get_Info(){
    return infos;
}

tSTAT toolflange::Get_State(){
    return states;
}

tGENS toolflange::Get_General(){
    return generals;
}

bool toolflange::Set_ConnectionTimerUp(unsigned char up_cnt){
    infos.connection_timer += up_cnt;
    if(infos.connection_timer >= 200){
        infos.connection_timer = 200;
        if(infos.connection_flag){
            rb_common::log_push(LogLevel::Warning, "TFB DISCON", P_NAME);
        }
        infos.connection_flag = false;
    }

    return infos.connection_flag;
}

bool toolflange::Get_ConnectionFlag(){
    return infos.connection_flag;
}

void toolflange::Set_Version(int t_version){
    infos.firmware_version = t_version;
}

void toolflange::Set_ClearGeneralData(){
    generals.gen_data = 0;
    generals.gen_sub_type = 0;
    generals.gen_type = 0;
    generals.gen_is_new = false;
}

tGENS toolflange::Get_GeneralData(){
    return generals;
}

CAN_MSG toolflange::CmdRequestVersion(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 20;
    can_m.dlc = 1;
    return can_m;
}

CAN_MSG toolflange::CmdPowerControl(int voltage){
    if(voltage < 0)     voltage = states.voltage;
    if(voltage != 12 && voltage != 24) voltage = 0;
    
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 1;// POWEr
    can_m.data[1] = voltage & 0xFF;
    can_m.dlc = 2;
    return can_m;
}

CAN_MSG toolflange::CmdDigitalOutput_Single(int port, int onoff){
    if(port != 1)   port = 0;
    if(onoff < 0)   onoff = states.dout[port] * 1000;
    if(onoff != 0)  onoff = 1000;
    
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 2;// DOUT
    can_m.data[1] = port & 0xFF;
    can_m.data[2] = (onoff >> 0) & 0xFF;
    can_m.data[3] = (onoff >> 8) & 0xFF;
    can_m.dlc = 4;
    return can_m;
}

CAN_MSG toolflange::CmdDigitalOutput_Multi(int ch0, int ch1){
    if(ch0 < 0)     ch0 = states.dout[0] * 1000;
    if(ch1 < 0)     ch1 = states.dout[0] * 1000;

    if(ch0 != 0)    ch0 = 1000;
    if(ch1 != 0)    ch1 = 1000;

    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 2;// DOUT
    can_m.data[1] = (ch0 >> 0) & 0xFF;
    can_m.data[2] = (ch0 >> 8) & 0xFF;
    can_m.data[3] = (ch1 >> 0) & 0xFF;
    can_m.data[4] = (ch1 >> 8) & 0xFF;
    can_m.dlc = 5;
    return can_m;
}

CAN_MSG toolflange::CmdGMbus_Init(unsigned int parity, unsigned int buadrate, int readFrequncy){
    // readFrequncy : -1: default
    //                 0: 25Hz
    //                 1: 50Hz   
    //                 2: 75Hz
    //                 3: 125Hz
    //                 4: 200Hz
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_GMBUS_SET;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 0;//set
    can_m.data[1] = parity & 0xFF;
    can_m.data[2] = (buadrate >> 0) & 0xFF;
    can_m.data[3] = (buadrate >> 8) & 0xFF;
    can_m.data[4] = (buadrate >> 16) & 0xFF;
    if(readFrequncy >= 0 && readFrequncy <= 4){
        can_m.data[5] = readFrequncy & 0xFF;
        can_m.dlc = 6;
    }else{
        can_m.dlc = 5;
    }
    return can_m;
}

CAN_MSG toolflange::CmdGMbus_Read_List_Clear(){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_GMBUS_SET;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 1;//add
    can_m.dlc = 1;//list clear
    return can_m;
}

CAN_MSG toolflange::CmdGMbus_Read_List_Add(unsigned int target_id, unsigned int target_fc, unsigned int target_dlc, unsigned int target_address){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_GMBUS_SET;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 1;//add
    can_m.data[1] = target_id;
    can_m.data[2] = target_fc & 0xFF;
    can_m.data[3] = target_dlc;
    can_m.data[4] = (target_address >> 0) & 0xFF;
    can_m.data[5] = (target_address >> 8) & 0xFF;
    can_m.dlc = 6;
    return can_m;
}

CAN_MSG toolflange::CmdGMbus_Read_Single(unsigned int target_id, unsigned int target_fc, unsigned int target_dlc, unsigned int target_address, unsigned int timeout_cnt){
    // target_dlc = 1 or 2
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_GMBUS_SET;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = 10;//read single
    can_m.data[1] = target_id;
    can_m.data[2] = target_fc & 0xFF;
    can_m.data[3] = target_dlc;
    can_m.data[4] = (target_address >> 0) & 0xFF;
    can_m.data[5] = (target_address >> 8) & 0xFF;
    can_m.data[6] = timeout_cnt;
    can_m.dlc = 7;
    return can_m;
}

CAN_MSG toolflange::CmdGMbus_Write_Single(unsigned int target_id, unsigned int target_fc, unsigned int target_address, unsigned int target_data){
    // target_fc
    // 6 or 16
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_GMBUS_WRITE;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = target_id;
    can_m.data[1] = target_fc;
    can_m.data[2] = (target_address >> 0) & 0xFF;
    can_m.data[3] = (target_address >> 8) & 0xFF;
    can_m.data[4] = (target_data >> 0) & 0xFF;
    can_m.data[5] = (target_data >> 8) & 0xFF;
    can_m.dlc = 6;
    return can_m;
}

CAN_MSG toolflange::CmdGMbus_Write_Multi(unsigned int target_id, unsigned int target_fc, unsigned int target_address, unsigned int target_data_0, unsigned int target_data_1){
    // target_fc
    // 6 or 16
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_GMBUS_WRITE;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = target_id;
    can_m.data[1] = target_fc;
    can_m.data[2] = (target_address >> 0) & 0xFF;
    can_m.data[3] = (target_address >> 8) & 0xFF;
    can_m.data[4] = (target_data_0 >> 0) & 0xFF;
    can_m.data[5] = (target_data_0 >> 8) & 0xFF;
    can_m.data[6] = (target_data_1 >> 0) & 0xFF;
    can_m.data[7] = (target_data_1 >> 8) & 0xFF;
    can_m.dlc = 8;
    return can_m;
}