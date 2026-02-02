#ifndef MOTION_H
#define MOTION_H

#include "../rb_common/rmath.h"

namespace rb_motion {
    //--------------------------------------------------------------------------------------
    // Sector Move
    //--------------------------------------------------------------------------------------
    

    bool        initialize();
    bool        Model_Update();
    bool        Get_Is_Idle();
    bool        Get_Is_Motion_Idle();
    bool        Get_Is_Wrapper_Idle();
    void        Sync_Wrapper_to_Motion();

    
    std::tuple<int,  TARGET_INPUT>      Calc_J_Relative(TARGET_INPUT delta_);
    std::tuple<int,  TARGET_INPUT>      Calc_J_Relative(TARGET_INPUT delta_, TARGET_INPUT pin_);
    std::tuple<int,  TARGET_INPUT>      Calc_L_Relative(TARGET_INPUT delta_);
    std::tuple<int,  TARGET_INPUT>      Calc_L_Relative(TARGET_INPUT delta_, TARGET_INPUT pin_);

    std::tuple<int,  TARGET_INPUT>      Calc_Relative_Value(TARGET_INPUT delta_, TARGET_INPUT pin_, int move_type);
    std::tuple<int,  TARGET_INPUT>      Calc_Absoulte_Value(TARGET_INPUT pin_, int move_type);

    //--------------------------------------------------------------------------------------
    // Sector Motion
    //--------------------------------------------------------------------------------------
    enum class MotionMode           {MOVE_NONE, MOVE_J, MOVE_L, MOVE_JB, MOVE_LB, MOVE_XB, MOVE_CIR, MOVE_SERVO_J, MOVE_SERVO_L, MOVE_SPD_J, MOVE_SPD_L};
    std::tuple<VectorJd, VectorCd, double> Task_Motion_Handler(double System_DT, double External_Alpha, bool is_motion_break, double break_alpha);

    void        Set_Motion_Mode(MotionMode t_mode);
    MotionMode  Get_Motion_Mode();
    int         Get_Motion_Ex_Result();
    void        Set_Motion_q(unsigned int idx, double angle);
    VectorCd    Get_Motion_X();
    VectorJd    Get_Motion_J();
    void        Wait_Motion_Finish();

    int         Start_Approach_Process();

    int         Start_Motion_J(TARGET_INPUT tar, double vel_para, double acc_para, int mode);
    int         Start_Motion_J(TARGET_INPUT tar, VectorJd j_vel, VectorJd j_acc);

    int         Start_Motion_L(TARGET_INPUT tar, double vel_para, double acc_para, int mode);
    int         Start_Motion_L(TARGET_INPUT tar, double p_vel, double p_acc, double o_vel, double o_acc);

    int         Start_Motion_JB_Clear();
    int         Start_Motion_JB_Add(TARGET_INPUT tar, double vel_para, double acc_para, int mode,   int blend_option, double blend_parameter);
    int         Start_Motion_JB_Add(TARGET_INPUT tar, VectorJd j_vel, VectorJd j_acc,               int blend_option, double blend_parameter);
    int         Start_Motion_JB();

    int         Start_Motion_LB_Clear();
    int         Start_Motion_LB_Add(TARGET_INPUT tar, double vel_para, double acc_para, int mode,               int point_type, double blend_parameter);
    int         Start_Motion_LB_Add(TARGET_INPUT tar, double p_vel, double p_acc, double o_vel, double o_acc,   int point_type, double blend_parameter);
    int         Start_Motion_LB(int rot_mode, int filter_window);

    int         Start_Motion_XB_Clear();
    int         Start_Motion_XB_Add(TARGET_INPUT tar, double g_vel_para, double g_acc_para, int mode, int blend_option, double blend_parameter, int xb_move_type);
    int         Start_Motion_XB_Add_J(TARGET_INPUT tar, double j_vel_para, double j_acc_para, int mode, int blend_option, double blend_parameter);
    int         Start_Motion_XB_Add_J(TARGET_INPUT tar, VectorJd j_vel, VectorJd j_acc, int blend_option, double blend_parameter);
    int         Start_Motion_XB_Add_L(TARGET_INPUT tar, double l_vel_para, double l_acc_para, int mode, int blend_option, double blend_parameter);
    int         Start_Motion_XB_Add_L(TARGET_INPUT tar, double p_vel, double p_acc, double o_vel, double o_acc, int blend_option, double blend_parameter);
    int         Start_Motion_XB(int mode);

    int         Start_Motion_CIR_AXIS(TARGET_INPUT center, TARGET_INPUT axis, double angle, int rot_mode, double l_vel_alpha, double l_acc_alpha, int radi_mode, double radi_para);
    int         Start_Motion_CIR_AXIS(TARGET_INPUT center, TARGET_INPUT axis, double angle, int rot_mode, double p_vel, double p_acc, double o_vel, double o_acc, int radi_mode, double radi_para);

    int         Start_Motion_CIR_3PNTS(TARGET_INPUT via, TARGET_INPUT tar, int rot_mode, double l_vel_alpha, double l_acc_alpha, int radi_mode, double radi_para);
    int         Start_Motion_CIR_3PNTS(TARGET_INPUT via, TARGET_INPUT tar, int rot_mode, double p_vel, double p_acc, double o_vel, double o_acc, int radi_mode, double radi_para);

    int         Start_Motion_SERVO_J(TARGET_INPUT tar, double t1, double t2, double gain, double filter);
    int         Start_Motion_SERVO_L(TARGET_INPUT tar, double t1, double t2, double gain, double filter);

    int         Start_Motion_SPEED_J(TARGET_INPUT tar, double j_acc_alpha);
    int         Start_Motion_SPEED_L(TARGET_INPUT tar, double l_acc_alpha);

    int         Set_Motion_Shift(int shift_no, int shift_mode, TARGET_INPUT shift_target);    
    int         Clear_Motion_Shift(int shift_no);

    //--------------------------------------------------------------------------------------
    // Sector Wrapper
    //--------------------------------------------------------------------------------------

    std::tuple<VectorJd, VectorJd, VectorJd, double> Task_Wrapper_Handler(double System_DT, double External_Alpha, VectorJd pre_motion_q, VectorCd pre_motion_x);
    void        Set_Wrapper_q(unsigned int idx, double angle);
    std::array<float, NO_OF_CARTE>    Get_Wrapper_X();
    std::array<float, NO_OF_CARTE>    Get_Wrapper_X_User();
    std::array<float, NO_OF_JOINT>    Get_Wrapper_J();

    int         Stop_Wrapper_All();

    int         Start_Wrapper_Tcp_Weaving(float magnitude, float time);
    int         Stop_Wrapper_Tcp_Weaving();

    int         Start_Wrapper_Base_Conv(float vel_x, float vel_y, float vel_z);
    int         Stop_Wrapper_Base_Conv();

}
#endif // MOTION_H
