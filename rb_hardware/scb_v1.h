#ifndef SCB_V1_H
#define SCB_V1_H

#include "../rb_common/canobserver.h"
#include "lan2can.h"

enum SF_SETTING_LIST{
    SFSET_NONE = 0,
    SFSET_BOUND_SLP_UP_J0,//
    SFSET_BOUND_SLP_UP_J1,//
    SFSET_BOUND_SLP_UP_J2,//
    SFSET_BOUND_SLP_UP_J3,//
    SFSET_BOUND_SLP_UP_J4,//
    SFSET_BOUND_SLP_UP_J5,//
    SFSET_BOUND_SLP_DOWN_J0,//
    SFSET_BOUND_SLP_DOWN_J1,//
    SFSET_BOUND_SLP_DOWN_J2,//
    SFSET_BOUND_SLP_DOWN_J3 = 10,//
    SFSET_BOUND_SLP_DOWN_J4,//
    SFSET_BOUND_SLP_DOWN_J5,//
    SFSET_BOUND_SLP_CNT,
    SFSET_BOUND_SLS_UP_J0,//
    SFSET_BOUND_SLS_UP_J1,//
    SFSET_BOUND_SLS_UP_J2,//
    SFSET_BOUND_SLS_UP_J3,//
    SFSET_BOUND_SLS_UP_J4,//
    SFSET_BOUND_SLS_UP_J5,//
    SFSET_BOUND_SLS_DOWN_J0 = 20,//
    SFSET_BOUND_SLS_DOWN_J1,//
    SFSET_BOUND_SLS_DOWN_J2,//
    SFSET_BOUND_SLS_DOWN_J3,//
    SFSET_BOUND_SLS_DOWN_J4,//
    SFSET_BOUND_SLS_DOWN_J5,//
    SFSET_BOUND_SLS_CNT,
    SFSET_BOUND_SLA_UP_J0,//
    SFSET_BOUND_SLA_UP_J1,//
    SFSET_BOUND_SLA_UP_J2,//
    SFSET_BOUND_SLA_UP_J3 = 30,//
    SFSET_BOUND_SLA_UP_J4,//
    SFSET_BOUND_SLA_UP_J5,//
    SFSET_BOUND_SLA_DOWN_J0,//
    SFSET_BOUND_SLA_DOWN_J1,//
    SFSET_BOUND_SLA_DOWN_J2,//
    SFSET_BOUND_SLA_DOWN_J3,//
    SFSET_BOUND_SLA_DOWN_J4,//
    SFSET_BOUND_SLA_DOWN_J5,//
    SFSET_BOUND_SLA_CNT,
    SFSET_BOUND_SLI_UP_J0 = 40,//
    SFSET_BOUND_SLI_UP_J1,//
    SFSET_BOUND_SLI_UP_J2,//
    SFSET_BOUND_SLI_UP_J3,//
    SFSET_BOUND_SLI_UP_J4,//
    SFSET_BOUND_SLI_UP_J5,//
    SFSET_BOUND_SLI_DOWN_J0,//
    SFSET_BOUND_SLI_DOWN_J1,//
    SFSET_BOUND_SLI_DOWN_J2,//
    SFSET_BOUND_SLI_DOWN_J3,//
    SFSET_BOUND_SLI_DOWN_J4 = 50,//
    SFSET_BOUND_SLI_DOWN_J5,//
    SFSET_BOUND_SLI_CNT,
    SFSET_BOUND_SLT_UP_J0,//
    SFSET_BOUND_SLT_UP_J1,//
    SFSET_BOUND_SLT_UP_J2,//
    SFSET_BOUND_SLT_UP_J3,//
    SFSET_BOUND_SLT_UP_J4,//
    SFSET_BOUND_SLT_UP_J5,//
    SFSET_BOUND_SLT_DOWN_J0,//
    SFSET_BOUND_SLT_DOWN_J1 = 60,//
    SFSET_BOUND_SLT_DOWN_J2,//
    SFSET_BOUND_SLT_DOWN_J3,//
    SFSET_BOUND_SLT_DOWN_J4,//
    SFSET_BOUND_SLT_DOWN_J5,//
    SFSET_BOUND_SLT_CNT,
    SFSET_BOUND_ECI_CNT,
    SFSET_BOUND_ECC_CNT,
    SFSET_BOUND_ECT_CNT,
    SFSET_BOUND_TCP_XMIN,
    SFSET_BOUND_TCP_XMAX = 70,//
    SFSET_BOUND_TCP_YMIN,//
    SFSET_BOUND_TCP_YMAX,//
    SFSET_BOUND_TCP_ZMIN,//
    SFSET_BOUND_TCP_ZMAX,//
    SFSET_BOUND_TCP_VEL,//
    SFSET_BOUND_COUNT_SS1,
    SFSET_BOUND_COUNT_SS2,
    SFSET_BOUND_COUNT_CBPL,
    SFSET_BOUND_COUNT_TPL,
    SFSET_BOUND_COUNT_TSL = 80,
    SFSET_BOUND_COUNT_RESET,
    SFSET_BOUND_COUNT_SWE,
    SFSET_BOUND_COUNT_TMP,
    SFSET_BOUND_VOLTAGE_UP,//
    SFSET_BOUND_VOLTAGE_DOWN,//
    SFSET_BOUND_SLOWON_UP,//
    SFSET_BOUND_SLOWON_DOWN,//
    SFSET_BOUND_CURRENT_VAL,//
    SFSET_INFO_L1,
    SFSET_INFO_L2 = 90,
    SFSET_INFO_L3,
    SFSET_INFO_L4,
    SFSET_INFO_L5,
    SFSET_INFO_L6,
    SFSET_INFO_L7,
    SFSET_INFO_LX,
    SFSET_INFO_LY,
    SFSET_INFO_LZ,
    SFSET_CONFIG_SF_ONOFF,
    SFSET_WALL_0_CX = 100,
    SFSET_WALL_0_CY,
    SFSET_WALL_0_CZ,
    SFSET_WALL_0_NX,
    SFSET_WALL_0_NY,
    SFSET_WALL_0_NZ,
    SFSET_WALL_0_OPT,
    SFSET_WALL_1_CX,
    SFSET_WALL_1_CY,
    SFSET_WALL_1_CZ,
    SFSET_WALL_1_NX = 110,
    SFSET_WALL_1_NY,
    SFSET_WALL_1_NZ,
    SFSET_WALL_1_OPT,
    SFSET_WALL_2_CX,
    SFSET_WALL_2_CY,
    SFSET_WALL_2_CZ,
    SFSET_WALL_2_NX,
    SFSET_WALL_2_NY,
    SFSET_WALL_2_NZ,
    SFSET_WALL_2_OPT = 120,
    SFSET_WALL_3_CX,
    SFSET_WALL_3_CY,
    SFSET_WALL_3_CZ,
    SFSET_WALL_3_NX,
    SFSET_WALL_3_NY,
    SFSET_WALL_3_NZ,
    SFSET_WALL_3_OPT,
    SFSET_SOS_LIMCNT,
    SFSET_SW_ROLE,
    SFSET_NUMBER
};

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

typedef union{
    unsigned int            raw;
    struct{
        unsigned char       on_dummy1 : 3;
        unsigned char       on_SLP : 1;
        unsigned char       on_SLS : 1;
        unsigned char       on_SLA : 1;
        unsigned char       on_SLI : 1;
        unsigned char       on_SLT : 1;

        unsigned char       on_TPL : 1;
        unsigned char       on_TSL : 1;
        unsigned char       on_CBP : 1;
        unsigned char       on_EM1 : 1;
        unsigned char       on_EM2 : 1;
        unsigned char       on_PRS : 1;
        unsigned char       on_HSS : 1;
        unsigned char       on_SSS : 1;

        unsigned char       on_ECI : 1;
        unsigned char       on_ECC : 1;
        unsigned char       on_ECT : 1;
        unsigned char       on_SWE : 1;
        unsigned char       on_TMP : 1;
        unsigned char       on_MEM : 1;
        unsigned char       on_FRM : 1;

        unsigned short      on_dummy2 : 9;
    };
}SAFETY_ONOFF_STRUCTURE;

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

struct iCANS{
    int                         CAN_CH[2];
    int                         CAN_ID_SSX[2];
    int                         CAN_ID_CMD[2];
    int                         CAN_ID_STA[2];
    int                         CAN_ID_RET[2];
};

struct iINFOS{
    bool                        configure_done[2];

    bool                        connection_flag[2];
    unsigned short              connection_timer[2];

    bool                        config_flag[2];
    unsigned char               config_addr[2];
    int                         config_data[2];

    int                         version[2];
};

class scb_v1 : public ICANObserver
{
public:
    scb_v1();
    ~scb_v1();

    void onCANMessage(int ch, int id, const unsigned char* data, int dlc) override;

    void   Clear_Infos();
    iINFOS Get_Infos();

    bool Set_ConnectionTimerUp(unsigned int sf_no, unsigned char up_cnt);
    bool Get_ConnectionFlag(unsigned int sf_no);

    void Set_ConfigDoneFlag(int sf_no, bool true_or_false);

    CAN_MSG Cmd_Setting(unsigned int sf_no, unsigned int sf_register, unsigned int sf_data);

    SAFETY_MCU_SHARED_DATA_STRUC safety_mcu[2];

private:
    iCANS       cans;
    iINFOS      infos;
};
#endif // LAN2CAN_H
