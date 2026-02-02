#include "scb_v1.h"

#include "common.h"

#define P_NAME  "SCB_V1"

scb_v1::scb_v1()
{
    cans.CAN_CH[0] = 0;
    cans.CAN_CH[1] = 1;

    cans.CAN_ID_SSX[0] = 0x04;
    cans.CAN_ID_SSX[1] = 0x05;

    cans.CAN_ID_CMD[0] = 0x407;
    cans.CAN_ID_CMD[1] = 0x417;

    cans.CAN_ID_STA[0] = 0x3F5;
    cans.CAN_ID_STA[1] = 0x3F9;

    cans.CAN_ID_RET[0] = 0x3F7;
    cans.CAN_ID_RET[1] = 0x3FB;

    for(int k = 0; k <2; ++ k){
        infos.configure_done[k] = false;
        infos.connection_timer[k] = 1000;
        infos.connection_flag[k] = false;
    }
}

scb_v1::~scb_v1(){
    ;
}

void scb_v1::onCANMessage(int ch, int id, const unsigned char* data, int dlc) {
    // id 기반으로 본인의 메시지만 필터링하고 처리
    (void)ch;
    (void)id;
    (void)dlc;
    (void)data;

    if(id == cans.CAN_ID_SSX[ch]){
        int type = data[0];
        int time = data[1];
        std::cout<<"SCB"<<ch<<" issue "<<type<<std::endl;
    }else if(id == cans.CAN_ID_STA[ch]){
        infos.connection_timer[ch] = 0;
        if(!infos.connection_flag[ch]){
            LOG_INFO("SCB CON " + std::to_string(ch));
        }
        infos.connection_flag[ch] = true;
        //std::cout << P_NAME << " can : "<<ch<<std::endl;
    }else if(id == cans.CAN_ID_RET[ch]){
        if(data[0] == 1){
            infos.config_addr[ch] = data[1];
            infos.config_data[ch] = data[3] | data[4] << 8 | data[5] << 16 | data[6] << 24;
            infos.config_flag[ch] = true;
        }else if(data[0] == 2){
            infos.version[ch] = data[2];
        }
    }
}

void scb_v1::Clear_Infos(){
    for(int k = 0; k < 2; ++k){
        infos.config_flag[k] = false;
        infos.config_data[k] = 0;
        infos.config_addr[k] = 0;
    }
}

iINFOS scb_v1::Get_Infos(){
    return infos;
}

bool scb_v1::Set_ConnectionTimerUp(unsigned int sf_no, unsigned char up_cnt){
    infos.connection_timer[sf_no] += up_cnt;
    if(infos.connection_timer[sf_no] >= 1000){
        infos.connection_timer[sf_no] = 1000;
        if(infos.connection_flag[sf_no]){
            LOG_WARNING("SCB DISCON " + std::to_string(sf_no));
        }
        infos.configure_done[sf_no] = false;
        infos.connection_flag[sf_no] = false;
    }
    return infos.connection_flag[sf_no];
}

bool scb_v1::Get_ConnectionFlag(unsigned int sf_no){
    return infos.connection_flag[sf_no];
}

void scb_v1::Set_ConfigDoneFlag(int sf_no, bool true_or_false){
    if(sf_no < 0){
        infos.configure_done[0] = true_or_false;
        infos.configure_done[1] = true_or_false;
    }else{
        infos.configure_done[sf_no] = true_or_false;
    }
}

CAN_MSG scb_v1::Cmd_Setting(unsigned int sf_no, unsigned int sf_register, unsigned int sf_data){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_CMD[sf_no];
    can_m.channel = cans.CAN_CH[sf_no];
    can_m.data[0] = 0;//write
    can_m.data[1] = sf_register;
    can_m.data[2] = 2;//N/A
    can_m.data[3] = (sf_data >> 0) & 0xFF;
    can_m.data[4] = (sf_data >> 8) & 0xFF;
    can_m.data[5] = (sf_data >> 16) & 0xFF;
    can_m.data[6] = (sf_data >> 24) & 0xFF;
    can_m.dlc = 7;
    return can_m;
}