#ifndef SCB_V1_H
#define SCB_V1_H

#include "../rb_common/canobserver.h"

typedef union{
    unsigned char raw;
    struct{
        unsigned char   dummy : 2;
        unsigned char   is_PC_sf_on : 1;
        unsigned char   is_PC_ref_on : 1;
        unsigned char   is_PC_teaching : 1;
        unsigned char   is_PC_coll_detect : 1;
        unsigned char   is_PC_prog_run : 1;
        unsigned char   is_PC_find_home : 1;
    };

}SAFETY_PC_STAT_STRUC;

typedef union{
    unsigned char raw;
    struct{
        unsigned char   line_A_val : 6;
        unsigned char   in_V_stat : 1;
        unsigned char   out_V_stat : 1;
    };
}SAFETY_UART_STAT_STRUC;

struct SAFETY_PORT_STATUS_STRUC{
    unsigned char       is_EMS1_pressed;
    unsigned char       is_EMS2_pressed;
    unsigned char       is_PRS_pressed;
    unsigned char       is_HSS_pressed;
    unsigned char       is_SSS_pressed;
    unsigned char       is_FET_out;
    unsigned char       is_Line_1_out;
    unsigned char       is_Line_2_out;
};

typedef union{
    unsigned int raw;
    struct{
        unsigned char   pmode : 3;
        unsigned char   is_SLP : 1;//sto
        unsigned char   is_SLS : 1;//sto
        unsigned char   is_SLA : 1;//sto
        unsigned char   is_SLI : 1;//sto
        unsigned char   is_SLT : 1;//sto

        unsigned char   is_TPL : 1;//sto
        unsigned char   is_TSL : 1;//sto
        unsigned char   is_CBP : 1;//sto
        unsigned char   is_EM1 : 1;//ss1 - > sto
        unsigned char   is_EM2 : 1;//ss1 - > sto
        unsigned char   is_PRS : 1;//ss1 - > sto
        unsigned char   is_HSS : 1;//sto
        unsigned char   is_SSS : 1;//ss2 - > sos

        unsigned char   is_ECI : 1;//sto
        unsigned char   is_ECC : 1;//sto
        unsigned char   is_ECT : 1;//sto
        unsigned char   is_SWE : 1;//sto
        unsigned char   is_TMP : 1;//sto
        unsigned char   is_MEM : 1;//sto
        unsigned char   is_FRM : 1;//sto

        unsigned short  DUMMY : 9;
    };
}SAFETY_STATUS_STRUC;

struct SAFETY_MCU_SHARED_DATA_STRUC{
    SAFETY_PORT_STATUS_STRUC    status_port;
    SAFETY_PC_STAT_STRUC        status_pc;
    SAFETY_STATUS_STRUC         status_now;
    SAFETY_STATUS_STRUC         status_old;
    SAFETY_UART_STAT_STRUC      status_uart;
    unsigned char               is_new_data;
    unsigned char               is_check_new_data;
    unsigned char               version;
    unsigned char               general_data_flag;
    unsigned char               general_data_Bytes[8];
    unsigned char               config_flag;
    unsigned char               config_addr;
    int                         config_data;
    float                       out_max_vol_1;
    float                       out_max_vol_2;

};

class scb_v1 : public ICANObserver
{
public:
    scb_v1();
    ~scb_v1();

    void onCANMessage(int ch, int id, const unsigned char* data, int dlc) override;

    SAFETY_MCU_SHARED_DATA_STRUC safety_mcu[2];

private:
    ;
};
#endif // LAN2CAN_H
