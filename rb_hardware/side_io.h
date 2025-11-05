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
    void Set_Aout(unsigned int p_no, float value);

    void Set_Din_Filter_Count(unsigned int p_no, int t_count);

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
};
#endif // SIDE_IO_H
