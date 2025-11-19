#ifndef TOOLFLANGE_H
#define TOOLFLANGE_H

#include "canobserver.h"
#include "lan2can.h"

#define TFB_NUM_DIN     6
#define TFB_NUM_DOUT    2
struct tSTAT{
    double  acc[3];
    double  gyro[3];

    int     button;
    int     din[TFB_NUM_DIN];
    int     dout[TFB_NUM_DOUT];

    int     voltage;
    int     rs485;
    int     temperature;

    
};

struct tINFO{
    int             firmware_version;
    bool            connection_flag;
    unsigned char   connection_timer;
};

struct tCANS{
    int             CAN_CH;
    int             CAN_ID_CMD;
    int             CAN_ID_GMBUS_SET;
    int             CAN_ID_GMBUS_WRITE;
    int             CAN_ID_GYR;
    int             CAN_ID_ACC;
    int             CAN_ID_485;
    int             CAN_ID_MBS_LEGACY_RX;
    int             CAN_ID_GMBUS_READ_SINGLE;
    int             CAN_ID_PAR;
    int             CAN_ID_GEN;
    int             CAN_ID_BOOT;   
};

struct tGENS{
    bool            gen_is_new;
    int             gen_type;
    int             gen_sub_type;
    int             gen_data;
};

class toolflange : public ICANObserver
{
public:
    toolflange(int ch);
    ~toolflange();

    void onCANMessage(int ch, int id, const unsigned char* data, int dlc) override;

    tINFO Get_Info();
    tSTAT Get_State();
    tGENS Get_General();

    bool Set_ConnectionTimerUp(unsigned char up_cnt);
    bool Get_ConnectionFlag();

    void Set_Version(int t_version);

    void Set_ClearGeneralData();
    tGENS Get_GeneralData();

    CAN_MSG CmdRequestVersion();
    CAN_MSG CmdPowerControl(int voltage);
    CAN_MSG CmdDigitalOutput_Single(int port, int onoff);
    CAN_MSG CmdDigitalOutput_Multi(int ch0, int ch1);

    CAN_MSG CmdGMbus_Init(unsigned int parity, unsigned int buadrate, int readFrequncy);
    CAN_MSG CmdGMbus_Read_List_Clear();
    CAN_MSG CmdGMbus_Read_List_Add(unsigned int target_id, unsigned int target_fc, unsigned int target_dlc, unsigned int target_address);
    CAN_MSG CmdGMbus_Read_Single(unsigned int target_id, unsigned int target_fc, unsigned int target_dlc, unsigned int target_address, unsigned int timeout_cnt);
    CAN_MSG CmdGMbus_Write_Single(unsigned int target_id, unsigned int target_fc, unsigned int target_address, unsigned int target_data);
    CAN_MSG CmdGMbus_Write_Multi(unsigned int target_id, unsigned int target_fc, unsigned int target_address, unsigned int target_data_0, unsigned int target_data_1);
    

private:
    tCANS       cans;
    tSTAT       states;
    tINFO       infos;

    tGENS       generals;
};
#endif // TOOLFLANGE_H
