#ifndef SHAREDDATA_H
#define SHAREDDATA_H

#include <iostream>
#include <array>
#include <mutex>  // 뮤텍스 헤더 추가
#include <dof.h>

#define NO_OF_SHM   3

typedef struct{
    int                                 heart_beat;

    std::array<float, NO_OF_JOINT>      joint_q_ref;
    std::array<float, NO_OF_JOINT>      joint_q_enc;
    std::array<float, NO_OF_JOINT>      joint_t_esti;
    std::array<float, NO_OF_JOINT>      joint_t_meas;
    std::array<float, NO_OF_JOINT>      joint_temper;

    std::array<float, NO_OF_CARTE>      carte_x_ref;
    std::array<float, NO_OF_CARTE>      carte_x_enc;

    int                                 userf_selection_no;
    std::array<float, NO_OF_CARTE>      userf_x_ref;

    int                                 tool_selection_no;
    char                                tool_name[64];
    float                               tool_tcp_x; 
    float                               tool_tcp_y; 
    float                               tool_tcp_z; 
    float                               tool_tcp_rx;
    float                               tool_tcp_ry;
    float                               tool_tcp_rz;
    float                               tool_com_m; 
    float                               tool_com_x; 
    float                               tool_com_y; 
    float                               tool_com_z;
    
    std::array<uint8_t, NO_OF_DIN>      cbox_digital_input;
    std::array<uint8_t, NO_OF_DOUT>     cbox_digital_output;
    std::array<float, NO_OF_AIN>        cbox_analog_input;   
    std::array<float, NO_OF_AOUT>       cbox_analog_output;

    std::array<uint8_t, NO_OF_DIN>      ex_digital_input;  
    std::array<uint8_t, NO_OF_DOUT>     ex_digital_output;
    std::array<float, NO_OF_AIN>        ex_analog_input;   
    std::array<float, NO_OF_AOUT>       ex_analog_output; 

    std::array<uint8_t, NO_OF_DIN>      tool_digital_input;    
    std::array<uint8_t, NO_OF_DOUT>     tool_digital_output;
    std::array<float, NO_OF_AIN>        tool_analog_input;
    std::array<float, NO_OF_AOUT>       tool_analog_output;
    float                               tool_voltage_output;

    int                                 motion_mode;
    int                                 motion_execution_result;
    float                               motion_speed_bar;
    int                                 motion_is_pause;

    uint8_t                             status_lan2can;
    uint8_t                             status_switch_emg;
    uint8_t                             status_power_out;
    uint8_t                             status_servo_num;
    uint8_t                             status_is_refon;
    uint8_t                             status_out_coll;
    uint8_t                             status_self_coll;
    uint8_t                             status_dt_mode;
} ST_SHM_M2F_STATE_CORE;

typedef struct{
    int         command_flag;
    float       command_f[20];
    char        command_char[256];
}ST_SHM_F2M_COMMAND;


typedef struct{
    ST_SHM_M2F_STATE_CORE   m2f_state_core;
    ST_SHM_F2M_COMMAND      f2m_command;
}ST_SMM_MANIPULATOR_UNIT;

typedef struct{
    ST_SMM_MANIPULATOR_UNIT robots[NO_OF_SHM];
}ST_SHARED_DATA;

namespace rb_shareddata {
    bool initialize(std::string SHM_name);
    void finalize();

    void copySharedDataToLocal();

    ST_SHARED_DATA* getGlobalShm(); // getter 함수
    ST_SHARED_DATA* getLocalShm();
}
#endif // SHAREDDATA_H
