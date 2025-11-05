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
    int             CAN_ID_GYR;
    int             CAN_ID_ACC;
    int             CAN_ID_485;
    int             CAN_ID_MBS;
    int             CAN_ID_GMB;
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


private:
    tCANS       cans;
    tSTAT       states;
    tINFO       infos;

    tGENS       generals;
};
#endif // TOOLFLANGE_H
