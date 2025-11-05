#ifndef MOTOR_H
#define MOTOR_H

#include <array>

#include "canobserver.h"
#include "lan2can.h"
#include "rmath.h"

typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;	// Nonius Error //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;   // Multiturn Error //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    SPI_EXT:1;
        unsigned    CUR_BIG:1;
        unsigned    CAN_ERR:1;
        unsigned    EST_TMP:1;  //modi240105 : wheter the estimated temperature > 127
        unsigned    CUR_SUM:1;  //modi240105 : wheter the 3phase sum 0
        unsigned    rsvd1:3;

        unsigned    rsvd2:8;
    }b;
    unsigned char B[4];
}mSTAT;

struct mPARA{
    double          para_reduction_rate;
    double          para_enc_resol;
    double          para_pulse_to_deg;

    double          para_torque_const;
    double          para_inertia;
    double          para_friction_a;
    double          para_friction_b;
    double          para_stiffness;
    double          para_r_q;

    double          para_shake_pulse;

    double          para_reco_vel;
    double          para_reco_acc;

    double          para_hwmax_vel;
    double          para_hwmax_acc;
    double          para_hwmax_torque_rept;
    double          para_hwmax_torque_momt;

    double          para_limit_torqueNm;
    double          para_limit_speedDeg;
    double          para_limit_angleDeg_Low;
    double          para_limit_angleDeg_Up;

    std::array<double, 2>   para_temperature_esti;
};

struct mINFO{
    int             encoder_pulse;
    double          encoder_deg;
    double          encoder_deg_prev;
    double          encoder_deg_vel;
    double          encoder_deg_vel_LPF;
    double          encoder_deg_vel_LPF_prev;
    double          encoder_deg_acc;
    double          encoder_deg_acc_LPF;
    double          encoder_deg_error;

    double          torque_mA;
    double          torque_Nm;
    double          torque_mA_movingFiltered;
    double          torque_Nm_movingFiltered;

    double          temperature_motor;
    double          temperature_board;

    int             firmware_version;
    int             type_num;
    int             type_mdr;
    int             type_blm;
    double          stat_last_enc;
    int             stat_uvw;
    int             stat_cur;
    int             stat_mul;
    int             stat_ram_enc;
    int             stat_ram_dqp;
    int             stat_sensors;

    bool            connection_flag;
    unsigned char   connection_timer;
};

struct mCANS{
    int             CAN_BNO;
    int             CAN_CH;    
    int             CAN_ID_BOOT;
    int             CAN_ID_CMD;
    int             CAN_ID_PAR;
    int             CAN_ID_STA;
    int             CAN_ID_REF;
    int             CAN_ID_ENC;
};

class motor : public ICANObserver
{
public:
    motor(int bno, int ch);
    ~motor();

    void onCANMessage(int ch, int id, const unsigned char* data, int dlc) override;
    void    Clear_States();
    mSTAT   Get_States();

    void    Clear_Infos(int mode);
    mINFO   Get_Infos();

    void    Set_Parameters(mPARA tPara);
    mPARA   Get_Parameters();

    void    Set_Version(int version);

    bool    Set_ConnectionTimerUp(unsigned char up_cnt);
    bool    Get_ConnectionFlag();

    void    Activation_Process_Start();
    void    Activation_Process_Update(double current_angle);
    std::tuple<double, double> Activation_Process_Stop();

    void Set_Last_Reference(double angle_deg, double torque_Nm, double fb_gain, double ff_gain);

    CAN_MSG CmdServoOn(double esti_torque_Nm);
    CAN_MSG CmdRequestVersion();
    CAN_MSG CmdRequestStatus();
    CAN_MSG CmdControl();
    CAN_MSG CmdControl(double position_deg, double torque_Nm, double fb_gain, double ff_gain);

    CAN_MSG CmdMakeErrorSumZero();

    


private:
    mPARA           parameters;
    mSTAT           states;
    mINFO           infos;
    mCANS           cans;

    double          last_ref_angle_deg;
    double          last_ref_torque_Nm;
    double          last_ref_fb_gain;
    double          last_ref_ff_gain;

    bool            activating_flag;
    double          activating_start_angle;
    double          activating_max_angle;
    double          activating_min_angle;
    
    double          vel_lpf_alpha;
    double          acc_lpf_alpha;

    rb_math::MovingAverage mA_filter;
    
};
#endif // MOTOR_H
