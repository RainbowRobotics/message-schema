#ifndef SIDE_IO_H
#define SIDE_IO_H

#include "../rb_common/canobserver.h"
#include "lan2can.h"
#include "rb_common/rmath.h"

#define NO_OF_AIN       4
#define NO_OF_AOUT      4
#define NO_OF_DIN       16
#define NO_OF_DOUT      16

struct sSTAT{
    double                  adc_raw[NO_OF_AIN];
    double                  dac_raw[NO_OF_AOUT];
    unsigned int            din_raw[NO_OF_DIN];
    unsigned int            dout_raw[NO_OF_DOUT];
    unsigned int            din_filt[NO_OF_DIN];
    
    bool                    adc_init_stat;
    bool                    dac_init_stat;
    bool                    din_init_stat;
    bool                    dout_init_stat;
};

struct sINFO{
    int             firmware_version;
    bool            connection_flag;
    unsigned char   connection_timer;
};

struct sCANS{
    int             CAN_CH;
    int             CAN_ID_CONTROL;
    int             CAN_ID_INPUT;
    int             CAN_ID_OUTPUT;
    int             CAN_ID_CMD;
    int             CAN_ID_GENERAL;
};

struct sGENS{
    bool            gen_is_new;
    int             gen_type;
    int             gen_data;
    int             gen_bytes[8];
};


class side_io : public ICANObserver
{
public:
    side_io(int ch);
    ~side_io();

    void onCANMessage(int ch, int id, const unsigned char* data, int dlc) override;    

    void Set_Version(int t_version);
    void Set_ClearGeneralData();

    sINFO Get_Info();
    sSTAT Get_State();
    sGENS Get_General();

    bool Set_ConnectionTimerUp(unsigned char up_cnt);

    void Set_Dout(unsigned int p_no, int value);
    int  Get_Dout(unsigned int p_no);
    void Set_Aout(unsigned int p_no, float value);

    void Set_Din_Filter_Count(unsigned int p_no, int t_count);

    void Set_Dout_Pulse(unsigned int p_no, unsigned char direction, float t1, float t2, float t3);
    bool Get_Dout_Pulse_State(unsigned int p_no);
    void Update_Dout_Pulse(float dt);

    CAN_MSG CmdRequestVersion();
    CAN_MSG CmdIOControl();
    CAN_MSG GetCurrentOutputState();

private:
    sCANS                   cans;
    sSTAT                   states;
    sINFO                   infos;
    sGENS                   generals;

    int                     desired_Dout[NO_OF_DOUT];
    float                   desired_Aout[NO_OF_AOUT];

    CAN_MSG                 current_output_state;

    rb_math::SignalFilter   sig_filter_din[NO_OF_DIN];

    bool                    pulse_Dout_OnOff[NO_OF_DOUT];
    unsigned char           pulse_Dout_Direction[NO_OF_DOUT];
    float                   pulse_Dout_Timer[NO_OF_DOUT];
    float                   pulse_Dout_T1[NO_OF_DOUT];
    float                   pulse_Dout_T2[NO_OF_DOUT];
    float                   pulse_Dout_T3[NO_OF_DOUT];
};
#endif // SIDE_IO_H
